#ifndef HW_ZPU_CPUDEVS_H
#define HW_ZPU_CPUDEVS_H

#include "target-zpu/cpu.h"

/* Definitions for ZPU CPU internal devices.  */

/* zpu_int.c */
void cpu_zpu_irq_init_cpu(CPUZPUState *env);

/* cputimer.c */
void cpu_zpu_clock_init(CPUZPUState *);

#endif
