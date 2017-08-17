/*
 * RISC-V emulation for qemu: main translation routines.
 *
 * Author: Sagar Karandikar, sagark@eecs.berkeley.edu
 *
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "disas/disas.h"
#include "tcg-op.h"
#include "exec/cpu_ldst.h"


#include "exec/helper-proto.h"
#include "exec/helper-gen.h"

#include "exec/log.h"

#include "instmap.h"

#define ZPU_DEBUG_DISAS 0

/* global register indices */
static TCGv_ptr cpu_env;
static TCGv_i32 cpu_gpr[32], cpu_pc, cond_eq, cond_lt;
static TCGv_i64 cpu_fpr[32]; /* assume F and D extensions */
static TCGv load_res;
#ifdef CONFIG_USER_ONLY
static TCGv_i32 cpu_amoinsn;
#endif

#include "exec/gen-icount.h"

typedef struct DisasContext {
    struct TranslationBlock *tb;
    target_ulong pc;
    target_ulong next_pc;
    uint32_t opcode;
    int singlestep_enabled;
    int mem_idx;
    int bstate;
} DisasContext;

static inline void kill_unknown(DisasContext *ctx, int excp);

enum {
    BS_NONE     = 0, /* When seen outside of translation while loop, indicates
                     need to exit tb due to end of page. */
    BS_STOP     = 1, /* Need to exit tb for syscall, sret, etc. */
    BS_BRANCH   = 2, /* Need to exit tb for branch, jal, etc. */
};


static const char * const regnames[] = {
  "r0 ", "r1  ", "r2  ", "r3  ", "r4  ", "r5 ",  "r6 ",  "r7  ",
  "r8 ", "r9 ", "r10 ", "r11 ", "r12 ", "r13 ", "r14 ", "r15 ",
  "r16 ", "r17 ", "r18 ", "r19 ", "r20 ", "r21 ", "r22 ", "r23 ",
  "r24 ", "r25 ", "r26 ", "r27 ", "r28 ", "r29 ", "r30 ", "r31 "
};

static const char * const fpr_regnames[] = {
  "ft0", "ft1", "ft2",  "ft3",  "ft4", "ft5", "ft6",  "ft7",
  "fs0", "fs1", "fa0",  "fa1",  "fa2", "fa3", "fa4",  "fa5",
  "fa6", "fa7", "fs2",  "fs3",  "fs4", "fs5", "fs6",  "fs7",
  "fs8", "fs9", "fs10", "fs11", "ft8", "ft9", "ft10", "ft11"
};

/* convert zpu funct3 to qemu memop for load/store */
static const int tcg_memop_lookup[8] = {
    [0 ... 7] = -1,
    [0] = MO_SB,
    [1] = MO_TESW,
    [2] = MO_TESL,
    [4] = MO_UB,
    [5] = MO_TEUW,
#ifdef TARGET_ZPU64
    [3] = MO_TEQ,
    [6] = MO_TEUL,
#endif
};

#ifdef TARGET_ZPU64
#define CASE_OP_32_64(X) case X: case glue(X, W)
#else
#define CASE_OP_32_64(X) case X
#endif


static inline void generate_exception(DisasContext *ctx, int excp)
{
    tcg_gen_movi_i32(cpu_pc, ctx->pc);
    TCGv_i32 helper_tmp = tcg_const_i32(excp);
    gen_helper_raise_exception(cpu_env, helper_tmp);
    tcg_temp_free_i32(helper_tmp);
}

static inline void generate_exception_mbadaddr(DisasContext *ctx, int excp)
{
    tcg_gen_movi_i32(cpu_pc, ctx->pc);
    TCGv_i32 helper_tmp = tcg_const_i32(excp);
    gen_helper_raise_exception_mbadaddr(cpu_env, helper_tmp, cpu_pc);
    tcg_temp_free_i32(helper_tmp);
}

/* unknown instruction */
static inline void kill_unknown(DisasContext *ctx, int excp)
{
    generate_exception(ctx, excp);
    ctx->bstate = BS_STOP;
}

static inline bool use_goto_tb(DisasContext *ctx, target_ulong dest)
{
    if (unlikely(ctx->singlestep_enabled)) {
        return false;
    }

#ifndef CONFIG_USER_ONLY
    return (ctx->tb->pc & TARGET_PAGE_MASK) == (dest & TARGET_PAGE_MASK);
#else
    return true;
#endif
}

static inline void gen_goto_tb(DisasContext *ctx, int n, target_ulong dest)
{
    if (use_goto_tb(ctx, dest)) {
        /* chaining is only allowed when the jump is to the same page */
        tcg_gen_goto_tb(n);
        tcg_gen_movi_tl(cpu_pc, dest);
        tcg_gen_exit_tb((uintptr_t)ctx->tb + n);
    } else {
        tcg_gen_movi_tl(cpu_pc, dest);
        if (ctx->singlestep_enabled) {
            gen_helper_raise_exception_debug(cpu_env);
        }
        tcg_gen_exit_tb(0);
    }
}

/* Wrapper for getting reg values - need to check of reg is zero since
 * cpu_gpr[0] is not actually allocated
 */
static inline void gen_get_gpr(TCGv_i32 t, int reg_num)
{
    if (reg_num == 0) {
        tcg_gen_movi_i32(t, 0);
    } else {
        tcg_gen_mov_i32(t, cpu_gpr[reg_num]);
    }
}
/* Wrapper for setting reg values - need to check of reg is zero since
 * cpu_gpr[0] is not actually allocated. this is more for safety purposes,
 * since we usually avoid calling the OP_TYPE_gen function if we see a write to
 * $zero
 */
static inline void gen_set_gpr(int reg_num_dst, TCGv_i32 t)
{
    if (reg_num_dst != 0) {
        tcg_gen_mov_i32(cpu_gpr[reg_num_dst], t);
    }
}

static inline void gen_get_eq(TCGv_i32 t)
{
    tcg_gen_mov_i32(t, cond_eq);
}
static inline void gen_set_eq(TCGv_i32 t)
{
    tcg_gen_mov_i32(cond_eq, t);
}

static inline void gen_get_lt(TCGv_i32 t)
{
    tcg_gen_mov_i32(t, cond_lt);
}
static inline void gen_set_lt(TCGv_i32 t)
{
    tcg_gen_mov_i32(cond_lt, t);
}


static void gen_arith(DisasContext *ctx, uint32_t opc)
{
    uint32_t inst = ctx->opcode;
    int rd = extract32(inst, 10, 5);
    int rs1 = extract32(inst, 5, 5);
    int rs2 = extract32(inst, 0, 5);

    TCGv_i32 source1, source2;
    source1 = tcg_temp_new_i32();
    source2 = tcg_temp_new_i32();
    gen_get_gpr(source1, rs1);
    gen_get_gpr(source2, rs2);

    switch (opc) {
    case OPC_ZPU_ADD:
        tcg_gen_add_i32(source1, source1, source2);
        break;
    case OPC_ZPU_SUB:
#if 1
    {
        TCGv_i32 cond_eq = tcg_temp_new_i32();
        TCGv_i32 cond_lt = tcg_temp_new_i32();
        tcg_gen_setcond_i32(TCG_COND_EQ, cond_eq, source1, source2);
        tcg_gen_setcond_i32(TCG_COND_LT, cond_lt, source1, source2);
        gen_set_eq(cond_eq);
        gen_set_lt(cond_lt);
        tcg_temp_free_i32(cond_eq);
        tcg_temp_free_i32(cond_lt);
    }
#endif
        tcg_gen_sub_i32(source1, source1, source2);
        break;
    case OPC_ZPU_MUL:
        tcg_gen_mul_i32(source1, source1, source2);
        break;
    case OPC_ZPU_AND:
        tcg_gen_and_i32(source1, source1, source2);
        break;
    case OPC_ZPU_OR:
        tcg_gen_or_i32(source1, source1, source2);
        break;
    case OPC_ZPU_XOR:
        tcg_gen_xor_i32(source1, source1, source2);
        break;
    case OPC_ZPU_SHL:
        tcg_gen_andi_i32(source2, source2, 0x1f);
        tcg_gen_shl_i32(source1, source1, source2);
        break;
    case OPC_ZPU_SHR:
        tcg_gen_andi_i32(source2, source2, 0x1f);
        tcg_gen_shr_i32(source1, source1, source2);
        break;
    default:
        kill_unknown(ctx, ZPU_EXCP_ILLEGAL_INST);
        break;
    }

    gen_set_gpr(rd, source1);
    tcg_temp_free_i32(source1);
    tcg_temp_free_i32(source2);
}

static void gen_arith_imm(DisasContext *ctx, uint32_t opc)
{
    uint32_t inst = ctx->opcode;
    int rd = extract32(inst, 21, 5);
    int rs1 = extract32(inst, 16, 5);
    target_long imm  = 0;

    switch (opc) {
    case OPC_ZPU_ADDI:
    case OPC_ZPU_SUBI:
    case OPC_ZPU_MUL:
        imm = sextract32(inst, 0, 16);
        break;
    case OPC_ZPU_ANDI:
    case OPC_ZPU_ORI:
    case OPC_ZPU_XORI:
        imm = extract32(inst, 0, 16);
        break;
    case OPC_ZPU_SHLI:
    case OPC_ZPU_SHRI:
        imm = extract32(inst, 0, 5);
        break;
    default: 
        kill_unknown(ctx, ZPU_EXCP_ILLEGAL_INST);
        break;
    }

    TCGv_i32 source1;
    source1 = tcg_temp_new_i32();
    gen_get_gpr(source1, rs1);

    switch (opc) {
    case OPC_ZPU_ADDI:
        tcg_gen_addi_i32(source1, source1, imm);
        break;
    case OPC_ZPU_SUBI:
        tcg_gen_subi_i32(source1, source1, imm);
        break;
    case OPC_ZPU_MULI:
        tcg_gen_muli_i32(source1, source1, imm);
        break;
    case OPC_ZPU_ANDI:
        tcg_gen_andi_i32(source1, source1, imm);
        break;
    case OPC_ZPU_ORI:
        tcg_gen_ori_i32(source1, source1, imm);
        break;
    case OPC_ZPU_XORI:
        tcg_gen_xori_i32(source1, source1, imm);
        break;
    case OPC_ZPU_SHLI:
        tcg_gen_shli_i32(source1, source1, imm);
        break;
    case OPC_ZPU_SHRI:
        tcg_gen_shri_i32(source1, source1, imm); 
        break;
    default:
        kill_unknown(ctx, ZPU_EXCP_ILLEGAL_INST);
        break;
    }

    gen_set_gpr(rd, source1);
    tcg_temp_free_i32(source1);
}

static void gen_mov(DisasContext *ctx, uint32_t opc)
{
    uint32_t inst = ctx->opcode;
    int rd = extract32(inst, 21, 5);
    int rs1 = extract32(inst, 16, 5);
    target_long imm  = extract32(inst, 0, 16);
    TCGv_i32 source1; 
    TCGv_i32 source2; 
    source1 = tcg_temp_new_i32();
    source2 = tcg_temp_new_i32();

    printf("gen_mov: rd = %d, rs = %d, imm = %x\n", rd, rs1, imm);

    switch (opc) {
    case OPC_ZPU_MOV:
        gen_get_gpr(source1, rs1);
        gen_set_gpr(rd, source1);
        break;
    case OPC_ZPU_MOVLO:
        gen_get_gpr(source1, rd);
        tcg_gen_andi_i32(source1, source1, 0x0000ffff);
        tcg_gen_movi_i32(source2, imm);
        tcg_gen_shli_i32(source2, source2, 16);
        tcg_gen_or_i32(source1, source1, source2);
        gen_set_gpr(rd, source1);
        break;
    case OPC_ZPU_MOVHI:
        gen_get_gpr(source1, rd);
        tcg_gen_andi_i32(source1, source1, 0xffff0000);
        tcg_gen_ori_i32(source1, source1, imm);
        gen_set_gpr(rd, source1);
        break;
    default: 
        kill_unknown(ctx, ZPU_EXCP_ILLEGAL_INST);
        break;
    }

    tcg_temp_free_i32(source1);
    tcg_temp_free_i32(source2);
}

static void gen_load(DisasContext *ctx, uint32_t opc)
{
    uint32_t inst = ctx->opcode;
    int rd = extract32(inst, 21, 5);
    int rs1 = extract32(inst, 16, 5);
    target_long imm = sextract32(inst, 0, 16);

    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    gen_get_gpr(t0, rs1);
    tcg_gen_addi_tl(t0, t0, imm);


    switch (opc) {
    case OPC_ZPU_LDB:
        tcg_gen_qemu_ld8u(t1, t0, ctx->mem_idx);
        break;
    case OPC_ZPU_LDW:
        tcg_gen_qemu_ld16u(t1, t0, ctx->mem_idx);
        break;
    case OPC_ZPU_LDD:
        tcg_gen_qemu_ld32u(t1, t0, ctx->mem_idx);
        break;
    default:
        kill_unknown(ctx, ZPU_EXCP_ILLEGAL_INST);
        break;
    }

    gen_set_gpr(rd, t1);
    tcg_temp_free(t0);
    tcg_temp_free(t1);
}

static void gen_store(DisasContext *ctx, uint32_t opc)
{
    uint32_t inst = ctx->opcode;
    int rs1 = extract32(inst, 21, 5);
    int rs2 = extract32(inst, 16, 5);
    target_long imm = sextract32(inst, 0, 16);

    TCGv t0 = tcg_temp_new();
    TCGv dat = tcg_temp_new();
    gen_get_gpr(t0, rs1);
    tcg_gen_addi_tl(t0, t0, imm);
    gen_get_gpr(dat, rs2);

    switch (opc) {
    case OPC_ZPU_STB:
        tcg_gen_qemu_st8(dat, t0, ctx->mem_idx);
        break;
    case OPC_ZPU_STW:
        tcg_gen_qemu_st16(dat, t0, ctx->mem_idx);
        break;
    case OPC_ZPU_STD:
        tcg_gen_qemu_st32(dat, t0, ctx->mem_idx);
        break;
    default:
        kill_unknown(ctx, ZPU_EXCP_ILLEGAL_INST);
        break;
    }

    tcg_temp_free(t0);
    tcg_temp_free(dat);
}

static void gen_branch(DisasContext *ctx, uint32_t opc)
{
    uint32_t inst = ctx->opcode;
    TCGLabel *l = gen_new_label();
    TCGv source1, source2;
    target_long bimm = sextract32(inst, 0, 26);
    source1 = tcg_temp_new();
    source2 = tcg_temp_new();
    gen_get_eq(source1);
    gen_get_lt(source2);


    switch (opc) {
    case OPC_ZPU_JMP:
        tcg_gen_brcondi_tl(TCG_COND_ALWAYS, source1, 0, l);
        break;
    case OPC_ZPU_JMPEQ:
        tcg_gen_brcondi_tl(TCG_COND_EQ, source1, 1, l);
        break;
    case OPC_ZPU_JMPNEQ:
        tcg_gen_brcondi_tl(TCG_COND_EQ, source1, 0, l);
        break;
    case OPC_ZPU_JMPGT:
        tcg_gen_brcondi_tl(TCG_COND_EQ, source2, 0, l);
        break;
    case OPC_ZPU_JMPLT:
        tcg_gen_brcondi_tl(TCG_COND_EQ, source2, 1, l);
        break;
    default:
        kill_unknown(ctx, ZPU_EXCP_ILLEGAL_INST);
        break;
    }

    gen_goto_tb(ctx, 0, ctx->next_pc);

    gen_set_label(l); /* branch taken */
    gen_goto_tb(ctx, 1, ctx->pc + bimm);

    tcg_temp_free(source1);
    tcg_temp_free(source2);
    ctx->bstate = BS_BRANCH;
}

static void gen_call_ret(DisasContext *ctx, uint32_t opc)
{
    uint32_t inst = ctx->opcode;
    int rs1 = extract32(inst, 21, 5);
    target_long imm = sextract32(inst, 0, 21);

    switch (opc) {
    case OPC_ZPU_CALL:
        if (rs1 != 0) {
            tcg_gen_movi_tl(cpu_gpr[rs1], ctx->next_pc);
        }

        gen_goto_tb(ctx, 0, ctx->pc + imm); /* must use this for safety */
        break;
    case OPC_ZPU_RET:
        gen_get_gpr(cpu_pc, rs1);
        tcg_gen_exit_tb(0);
        break;
    default:
        kill_unknown(ctx, ZPU_EXCP_ILLEGAL_INST);
        break;
    }        
    ctx->bstate = BS_BRANCH;
}

static void 
decode_zpu_inst(CPUZPUState *env, DisasContext *ctx)
{
    uint32_t op;
#if 0
    int rs1;
    int rs2;
    int rd;
    target_long imm;
#endif
    /* We do not do misaligned address check here: the address should never be
     * misaligned at this point. Instructions that set PC must do the check,
     * since epc must be the address of the instruction that caused us to
     * perform the misaligned instruction fetch */

    op = MASK_OP_MAJOR(ctx->opcode);
#if 0
    rs1 = GET_RS1(ctx->opcode);
    rs2 = GET_RS2(ctx->opcode);
    rd = GET_RD(ctx->opcode);
    imm = GET_IMM(ctx->opcode);
#endif

    switch (op) {
    case OPC_ZPU_NOP:
        /* NOP: Do Nothing */
        break;
    case OPC_ZPU_HALT:
#if 0
        printf("HALT!!\n");
        fflush(stdout);
        tcg_gen_exit_tb(0);
#endif
        ctx->bstate = BS_STOP;
        kill_unknown(ctx, ZPU_EXCP_EXIT);
        break;

        //tcg_gen_exit_tb(0);
        break;
    case OPC_ZPU_ADD:
    case OPC_ZPU_SUB:
    case OPC_ZPU_MUL:
    case OPC_ZPU_AND:
    case OPC_ZPU_OR:
    case OPC_ZPU_XOR:
    case OPC_ZPU_SHL:
    case OPC_ZPU_SHR:
        gen_arith(ctx, op);
        break;
    case OPC_ZPU_ADDI:
    case OPC_ZPU_SUBI:
    case OPC_ZPU_MULI:
    case OPC_ZPU_ANDI:
    case OPC_ZPU_ORI:
    case OPC_ZPU_XORI:
    case OPC_ZPU_SHLI:
    case OPC_ZPU_SHRI:
        gen_arith_imm(ctx, op);
        break;
    case OPC_ZPU_LDB:
    case OPC_ZPU_LDW:
    case OPC_ZPU_LDD:
    case OPC_ZPU_LDQ:
        gen_load(ctx, op);
        break;
    case OPC_ZPU_STB:
    case OPC_ZPU_STW:
    case OPC_ZPU_STD:
    case OPC_ZPU_STQ:
        gen_store(ctx, op);
        break;
    case OPC_ZPU_MOV:
    case OPC_ZPU_MOVHI:
    case OPC_ZPU_MOVLO:
    case OPC_ZPU_MOVSEL:
        gen_mov(ctx, op);
        break;
    case OPC_ZPU_JMP:
    case OPC_ZPU_JMPEQ:
    case OPC_ZPU_JMPNEQ:
    case OPC_ZPU_JMPGT:
    case OPC_ZPU_JMPLT:
         gen_branch(ctx, op); 
        break;
    case OPC_ZPU_CALL:
    case OPC_ZPU_RET:
        gen_call_ret(ctx, op);
        break;
        
    default:
        kill_unknown(ctx, ZPU_EXCP_ILLEGAL_INST);
        break;
    }
}




static void decode_opc(CPUZPUState *env, DisasContext *ctx)
{
        ctx->next_pc = ctx->pc + 4;
        //decode_RV32_64G(env, ctx);
        decode_zpu_inst(env, ctx);
        //********************************** JOEVTEST **************************
        printf("decode: 0x%x\n", ctx->opcode);
        zpu_cpu_dump_state(CPU(zpu_env_get_cpu(env)), stdout, fprintf, 0);
}

void gen_intermediate_code(CPUZPUState *env, TranslationBlock *tb)
{
    ZPUCPU *cpu = zpu_env_get_cpu(env);
    CPUState *cs = CPU(cpu);
    DisasContext ctx;
    target_ulong pc_start;
    target_ulong next_page_start;
    int num_insns;
    int max_insns;
    pc_start = tb->pc;
    next_page_start = (pc_start & TARGET_PAGE_MASK) + TARGET_PAGE_SIZE;
    ctx.pc = pc_start;

    /* once we have GDB, the rest of the translate.c implementation should be
       ready for singlestep */
    ctx.singlestep_enabled = cs->singlestep_enabled;

    ctx.tb = tb;
    ctx.bstate = BS_NONE;

    ctx.mem_idx = cpu_mmu_index(env, false);
    num_insns = 0;
    max_insns = tb->cflags & CF_COUNT_MASK;
    if (max_insns == 0) {
        max_insns = CF_COUNT_MASK;
    }
    if (max_insns > TCG_MAX_INSNS) {
        max_insns = TCG_MAX_INSNS;
    }
    gen_tb_start(tb);

    while (ctx.bstate == BS_NONE) {
        tcg_gen_insn_start(ctx.pc);
        num_insns++;

        if (unlikely(cpu_breakpoint_test(cs, ctx.pc, BP_ANY))) {
            tcg_gen_movi_tl(cpu_pc, ctx.pc);
            ctx.bstate = BS_BRANCH;
            gen_helper_raise_exception_debug(cpu_env);
            /* The address covered by the breakpoint must be included in
               [tb->pc, tb->pc + tb->size) in order to for it to be
               properly cleared -- thus we increment the PC here so that
               the logic setting tb->size below does the right thing.  */
            ctx.pc += 4;
            goto done_generating;
        }

        if (num_insns == max_insns && (tb->cflags & CF_LAST_IO)) {
            gen_io_start();
        }

        ctx.opcode = cpu_ldl_code(env, ctx.pc);
        decode_opc(env, &ctx);
        ctx.pc = ctx.next_pc;

        if (cs->singlestep_enabled) {
            break;
        }
        if (ctx.pc >= next_page_start) {
            break;
        }
        if (tcg_op_buf_full()) {
            break;
        }
        if (num_insns >= max_insns) {
            break;
        }
        if (singlestep) {
            break;
        }

    }
    if (tb->cflags & CF_LAST_IO) {
        gen_io_end();
    }
    if (cs->singlestep_enabled && ctx.bstate != BS_BRANCH) {
        if (ctx.bstate == BS_NONE) {
            tcg_gen_movi_tl(cpu_pc, ctx.pc);
        }
        gen_helper_raise_exception_debug(cpu_env);
    } else {
        switch (ctx.bstate) {
        case BS_STOP:
            gen_goto_tb(&ctx, 0, ctx.pc);
            break;
        case BS_NONE: /* handle end of page - DO NOT CHAIN. See gen_goto_tb. */
            tcg_gen_movi_tl(cpu_pc, ctx.pc);
            tcg_gen_exit_tb(0);
            break;
        case BS_BRANCH: /* ops using BS_BRANCH generate own exit seq */
        default:
            break;
        }
    }
done_generating:
    gen_tb_end(tb, num_insns);
    tb->size = ctx.pc - pc_start;
    tb->icount = num_insns;

#ifdef DEBUG_DISAS
    if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)
        && qemu_log_in_addr_range(pc_start)) {
        qemu_log("IN: %s\n", lookup_symbol(pc_start));
        log_target_disas(cs, pc_start, ctx.pc - pc_start, 0);
        qemu_log("\n");
    }
#endif
}

void zpu_cpu_dump_state(CPUState *cs, FILE *f, fprintf_function cpu_fprintf,
                         int flags)
{
    ZPUCPU *cpu = ZPU_CPU(cs);
    CPUZPUState *env = &cpu->env;
    int i;

    cpu_fprintf(f, "pc=0x" TARGET_FMT_lx "\n", env->pc);
    for (i = 0; i < 32; i++) {
        cpu_fprintf(f, " %s " TARGET_FMT_lx, regnames[i], env->gpr[i]);
        if ((i & 3) == 3) {
            cpu_fprintf(f, "\n");
        }
    }
    cpu_fprintf(f, " %s " TARGET_FMT_lx, "cc_eq", env->cc_eq);
    cpu_fprintf(f, " %s " TARGET_FMT_lx, "cc_lt", env->cc_lt);
    cpu_fprintf(f, "\n--------------------------------------\n");
#if 0
#ifndef CONFIG_USER_ONLY
    cpu_fprintf(f, " %s " TARGET_FMT_lx "\n", "MSTATUS ",
                env->mstatus);
    cpu_fprintf(f, " %s " TARGET_FMT_lx "\n", "MIP     ", env->mip);
    cpu_fprintf(f, " %s " TARGET_FMT_lx "\n", "MIE     ", env->mie);
#endif

    for (i = 0; i < 32; i++) {
        if ((i & 3) == 0) {
            cpu_fprintf(f, "FPR%02d:", i);
        }
        cpu_fprintf(f, " %s %016" PRIx64, fpr_regnames[i], env->fpr[i]);
        if ((i & 3) == 3) {
            cpu_fprintf(f, "\n");
        }
    }
#endif /* 0 */
}

void zpu_tcg_init(void)
{
    int i;
    static int inited;

    /* Initialize various static tables. */
    if (inited) {
        return;
    }

    cpu_env = tcg_global_reg_new_ptr(TCG_AREG0, "env");

    /* WARNING: cpu_gpr[0] is not allocated ON PURPOSE. Do not use it. */
    /* Use the gen_set_gpr and gen_get_gpr helper functions when accessing */
    /* registers, unless you specifically block reads/writes to reg 0 */
    TCGV_UNUSED(cpu_gpr[0]);
    for (i = 1; i < 32; i++) {
        cpu_gpr[i] = tcg_global_mem_new(cpu_env,
                             offsetof(CPUZPUState, gpr[i]), regnames[i]);
    }
    cond_eq = tcg_global_mem_new(cpu_env,
                                 offsetof(CPUZPUState, cc_eq), "cc_eq");
    cond_lt = tcg_global_mem_new(cpu_env,
                                 offsetof(CPUZPUState, cc_lt), "cc_lt");

    for (i = 0; i < 32; i++) {
        cpu_fpr[i] = tcg_global_mem_new_i64(cpu_env,
                             offsetof(CPUZPUState, fpr[i]), fpr_regnames[i]);
    }

    cpu_pc = tcg_global_mem_new(cpu_env, offsetof(CPUZPUState, pc), "pc");
    load_res = tcg_global_mem_new(cpu_env, offsetof(CPUZPUState, load_res),
                             "load_res");

#ifdef CONFIG_USER_ONLY
    cpu_amoinsn = tcg_global_mem_new_i32(cpu_env,
                    offsetof(CPUZPUState, amoinsn),
                    "amoinsn");
#endif
    inited = 1;
}
