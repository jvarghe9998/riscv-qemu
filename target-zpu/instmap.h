/*
 * RISC-V emulation for qemu: Instruction decode helpers
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

#define MASK_OP_MAJOR(op)  ((op & 0xfc000000) >> 26)
enum {
#if 0
    /* rv32i, rv64i, rv32m */
    OPC_RISC_LUI    = (0x37),
    OPC_RISC_AUIPC  = (0x17),
    OPC_RISC_JAL    = (0x6F),
    OPC_RISC_JALR   = (0x67),
    OPC_RISC_BRANCH = (0x63),
    OPC_RISC_LOAD   = (0x03),
    OPC_RISC_STORE  = (0x23),
    OPC_RISC_ARITH_IMM  = (0x13),
    OPC_RISC_ARITH      = (0x33),
    OPC_RISC_FENCE      = (0x0F),
    OPC_RISC_SYSTEM     = (0x73),

    /* rv64i, rv64m */
    OPC_RISC_ARITH_IMM_W = (0x1B),
    OPC_RISC_ARITH_W = (0x3B),

    /* rv32a, rv64a */
    OPC_RISC_ATOMIC = (0x2F),

    /* floating point */
    OPC_RISC_FP_LOAD = (0x7),
    OPC_RISC_FP_STORE = (0x27),

    OPC_RISC_FMADD = (0x43),
    OPC_RISC_FMSUB = (0x47),
    OPC_RISC_FNMSUB = (0x4B),
    OPC_RISC_FNMADD = (0x4F),

    OPC_RISC_FP_ARITH = (0x53),
#endif
    OPC_ZPU_NOP = 0,
    OPC_ZPU_HALT = 0x3f,
    OPC_ZPU_ADD = 0x1,
    OPC_ZPU_SUB  = 0x2,
    OPC_ZPU_MUL  = 0x3,
    OPC_ZPU_AND  = 0x4,
    OPC_ZPU_OR  = 0x5,
    OPC_ZPU_XOR  = 0x6,
    OPC_ZPU_SHL  = 0x7,
    OPC_ZPU_SHR  = 0x8,
    OPC_ZPU_ADDI  = 0x11,
    OPC_ZPU_SUBI  = 0x12,
    OPC_ZPU_MULI  = 0x13,
    OPC_ZPU_ANDI  = 0x14,
    OPC_ZPU_ORI  = 0x15,
    OPC_ZPU_XORI  = 0x16,
    OPC_ZPU_SHLI  = 0x17,
    OPC_ZPU_SHRI  = 0x18,
    OPC_ZPU_LDB  = 0x20,
    OPC_ZPU_LDW  = 0x21,
    OPC_ZPU_LDD  = 0x22,
    OPC_ZPU_LDQ  = 0x23,
    OPC_ZPU_STB  = 0x24,
    OPC_ZPU_STW  = 0x25,
    OPC_ZPU_STD  = 0x26,
    OPC_ZPU_STQ  = 0x27,
    OPC_ZPU_MOV  = 0x28,
    OPC_ZPU_MOVHI  = 0x29,
    OPC_ZPU_MOVLO  = 0x2a,
    OPC_ZPU_MOVSEL  = 0x2b,
    OPC_ZPU_JMP = 0x30,
    OPC_ZPU_JMPEQ  = 0x31,
    OPC_ZPU_JMPNEQ  = 0x32,
    OPC_ZPU_JMPGT  = 0x33,
    OPC_ZPU_JMPLT  = 0x34,
    OPC_ZPU_CALL  = 0x35,
    OPC_ZPU_RET  = 0x36,
};

#if 0
#define GET_B_IMM(inst) (extract32(inst, 8, 4) << 1)    \
                        | (extract32(inst, 25, 6) << 5) \
                        | (extract32(inst, 7, 1) << 11) \
                        | (sextract64(inst, 31, 1) << 12)

#define GET_STORE_IMM(inst) (extract32(inst, 7, 5))        \
                            | (sextract64(inst, 25, 7) << 5)

#define GET_JAL_IMM(inst) (extract32(inst, 21, 10) << 1)   \
                          | (extract32(inst, 20, 1) << 11) \
                          | (extract32(inst, 12, 8) << 12) \
                          | (sextract64(inst, 31, 1) << 20)

#define GET_RM(inst)   extract32(inst, 12, 3)
#define GET_RS3(inst)  extract32(inst, 27, 5)
#define GET_RS1(inst)  extract32(inst, 15, 5)
#define GET_RS2(inst)  extract32(inst, 20, 5)
#define GET_RD(inst)   extract32(inst, 7, 5)
#define GET_IMM(inst)  sextract64(inst, 20, 12)

/* RVC decoding macros */
#define GET_C_IMM(inst)             (extract32(inst, 2, 5) \
                                    | (sextract64(inst, 12, 1) << 5))
#define GET_C_ZIMM(inst)            (extract32(inst, 2, 5) \
                                    | (extract32(inst, 12, 1) << 5))
#define GET_C_ADDI4SPN_IMM(inst)    ((extract32(inst, 6, 1) << 2) \
                                    | (extract32(inst, 5, 1) << 3) \
                                    | (extract32(inst, 11, 2) << 4) \
                                    | (extract32(inst, 7, 4) << 6))
#define GET_C_ADDI16SP_IMM(inst)    ((extract32(inst, 6, 1) << 4) \
                                    | (extract32(inst, 2, 1) << 5) \
                                    | (extract32(inst, 5, 1) << 6) \
                                    | (extract32(inst, 3, 2) << 7) \
                                    | (sextract64(inst, 12, 1) << 9))
#define GET_C_LWSP_IMM(inst)        ((extract32(inst, 4, 3) << 2) \
                                    | (extract32(inst, 12, 1) << 5) \
                                    | (extract32(inst, 2, 2) << 6))
#define GET_C_LDSP_IMM(inst)        ((extract32(inst, 5, 2) << 3) \
                                    | (extract32(inst, 12, 1) << 5) \
                                    | (extract32(inst, 2, 3) << 6))
#define GET_C_SWSP_IMM(inst)        ((extract32(inst, 9, 4) << 2) \
                                    | (extract32(inst, 7, 2) << 6))
#define GET_C_SDSP_IMM(inst)        ((extract32(inst, 10, 3) << 3) \
                                    | (extract32(inst, 7, 3) << 6))
#define GET_C_LW_IMM(inst)          ((extract32(inst, 6, 1) << 2) \
                                    | (extract32(inst, 10, 3) << 3) \
                                    | (extract32(inst, 5, 1) << 6))
#define GET_C_LD_IMM(inst)          ((extract32(inst, 10, 3) << 3) \
                                    | (extract32(inst, 5, 2) << 6))
#define GET_C_J_IMM(inst)           ((extract32(inst, 3, 3) << 1) \
                                    | (extract32(inst, 11, 1) << 4) \
                                    | (extract32(inst, 2, 1) << 5) \
                                    | (extract32(inst, 7, 1) << 6) \
                                    | (extract32(inst, 6, 1) << 7) \
                                    | (extract32(inst, 9, 2) << 8) \
                                    | (extract32(inst, 8, 1) << 10) \
                                    | (sextract64(inst, 12, 1) << 11))
#define GET_C_B_IMM(inst)           ((extract32(inst, 3, 2) << 1) \
                                    | (extract32(inst, 10, 2) << 3) \
                                    | (extract32(inst, 2, 1) << 5) \
                                    | (extract32(inst, 5, 2) << 6) \
                                    | (sextract64(inst, 12, 1) << 8))
#define GET_C_SIMM3(inst)           extract32(inst, 10, 3)
#define GET_C_RD(inst)              GET_RD(inst)
#define GET_C_RS1(inst)             GET_RD(inst)
#define GET_C_RS2(inst)             extract32(inst, 2, 5)
#define GET_C_RS1S(inst)            (8 + extract32(inst, 7, 3))
#define GET_C_RS2S(inst)            (8 + extract32(inst, 2, 3))
#endif
