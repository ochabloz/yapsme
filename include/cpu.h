#ifndef CPU_H
#define CPU_H

#include <stdint.h>

void cpu_initialise(void);
void cpu_run(uint32_t nb_cycles);
uint32_t cpu_read_reg(uint8_t reg);
void cpu_write_reg(uint8_t reg, uint32_t val);

#endif
