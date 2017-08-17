/*
 * QEMU RISC-V CPU
 *
 * Author: Sagar Karandikar, sagark@eecs.berkeley.edu
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "qemu-common.h"
#include "migration/vmstate.h"

static inline void set_feature(CPUZPUState *env, int feature)
{
    env->features |= 1ULL << feature;
}

static void zpu_cpu_set_pc(CPUState *cs, vaddr value)
{
    ZPUCPU *cpu = ZPU_CPU(cs);
    CPUZPUState *env = &cpu->env;
    env->pc = value;
}

static void zpu_cpu_synchronize_from_tb(CPUState *cs, TranslationBlock *tb)
{
    ZPUCPU *cpu = ZPU_CPU(cs);
    CPUZPUState *env = &cpu->env;
    env->pc = tb->pc;
}

#ifdef CONFIG_USER_ONLY
static bool zpu_cpu_has_work(CPUState *cs)
{
    return 0;
}
#else
static bool zpu_cpu_has_work(CPUState *cs)
{
    ZPUCPU *cpu = ZPU_CPU(cs);
    CPUZPUState *env = &cpu->env;
    bool has_work = false;

    if (cs->interrupt_request & CPU_INTERRUPT_HARD) {
        int interruptno = cpu_zpu_hw_interrupts_pending(env);
        if (interruptno + 1) {
            has_work = true;
        }
    }

    return has_work;
}
#endif

void restore_state_to_opc(CPUZPUState *env, TranslationBlock *tb,
                          target_ulong *data)
{
    env->pc = data[0];
}

static void zpu_cpu_reset(CPUState *s)
{
    ZPUCPU *cpu = ZPU_CPU(s);
    ZPUCPUClass *mcc = ZPU_CPU_GET_CLASS(cpu);
    CPUZPUState *env = &cpu->env;
    CPUState *cs = CPU(cpu);

    mcc->parent_reset(s);
#ifndef CONFIG_USER_ONLY
    tlb_flush(s, 1);
    env->priv = PRV_M;
    env->mtvec = DEFAULT_MTVEC;
#endif
    env->pc = DEFAULT_RSTVEC;
    cs->exception_index = EXCP_NONE;
}

static void zpu_cpu_disas_set_info(CPUState *s, disassemble_info *info) {
    info->print_insn = print_insn_zpu;
}

static void zpu_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    ZPUCPUClass *mcc = ZPU_CPU_GET_CLASS(dev);


    cpu_reset(cs);
    qemu_init_vcpu(cs);

    mcc->parent_realize(dev, errp);
}

static void zpu_cpu_initfn(Object *obj)
{
    CPUState *cs = CPU(obj);
    ZPUCPU *cpu = ZPU_CPU(obj);
    CPUZPUState *env = &cpu->env;

    cs->env_ptr = env;
    cpu_exec_init(cs, &error_abort);

    if (tcg_enabled()) {
        zpu_tcg_init();
    }
}

static const VMStateDescription vmstate_zpu_cpu = {
    .name = "cpu",
    .unmigratable = 1,
};

static void zpu_cpu_class_init(ObjectClass *c, void *data)
{
    ZPUCPUClass *mcc = ZPU_CPU_CLASS(c);
    CPUClass *cc = CPU_CLASS(c);
    DeviceClass *dc = DEVICE_CLASS(c);

    mcc->parent_realize = dc->realize;
    dc->realize = zpu_cpu_realizefn;

    mcc->parent_reset = cc->reset;
    cc->reset = zpu_cpu_reset;

    cc->has_work = zpu_cpu_has_work;
    cc->do_interrupt = zpu_cpu_do_interrupt;
    cc->cpu_exec_interrupt = zpu_cpu_exec_interrupt;
    cc->dump_state = zpu_cpu_dump_state;
    cc->set_pc = zpu_cpu_set_pc;
    cc->synchronize_from_tb = zpu_cpu_synchronize_from_tb;
    cc->gdb_read_register = zpu_cpu_gdb_read_register;
    cc->gdb_write_register = zpu_cpu_gdb_write_register;
    cc->gdb_num_core_regs = 65;
    cc->gdb_stop_before_watchpoint = true;
    cc->disas_set_info = zpu_cpu_disas_set_info;
#ifdef CONFIG_USER_ONLY
    cc->handle_mmu_fault = zpu_cpu_handle_mmu_fault;
#else
    cc->do_unassigned_access = zpu_cpu_unassigned_access;
    cc->do_unaligned_access = zpu_cpu_do_unaligned_access;
    cc->get_phys_page_debug = zpu_cpu_get_phys_page_debug;
#endif
    /* For now, mark unmigratable: */
    cc->vmsd = &vmstate_zpu_cpu;

    /*
     * Reason: zpu_cpu_initfn() calls cpu_exec_init(), which saves
     * the object in cpus -> dangling pointer after final
     * object_unref().
     */
    dc->cannot_destroy_with_object_finalize_yet = true;
}

static const TypeInfo zpu_cpu_type_info = {
    .name = TYPE_ZPU_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(ZPUCPU),
    .instance_init = zpu_cpu_initfn,
    .abstract = false,
    .class_size = sizeof(ZPUCPUClass),
    .class_init = zpu_cpu_class_init,
};

static void zpu_cpu_register_types(void)
{
    type_register_static(&zpu_cpu_type_info);
}

type_init(zpu_cpu_register_types)
