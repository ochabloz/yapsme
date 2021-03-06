#ifndef CPU_H
#define CPU_H

#include <stdint.h>

#define CPU_SUCCESS   0x00
#define CPU_FAILURE   0x01

// register numbers for reading them through cpu_read_reg
#define CPU_PC   0x40
#define CPU_HI   0x41
#define CPU_LO   0x42

uint32_t cpu_initialise(void);
uint32_t cpu_desinit(void);

uint32_t cpu_execute(uint32_t instruction);

uint32_t cpu_run(uint32_t nb_cycles);
uint32_t cpu_read_reg(uint8_t reg);
uint32_t cpu_get_interrupt_status();
uint32_t cpu_get_interrupt_mask();
void cpu_set_interrupt_mask(uint32_t mask);

uint32_t cp0_read_reg(uint8_t reg);
void cp0_write_reg(uint8_t reg, uint32_t val);

#define CPU_REG_DELAY_ON 1
#define CPU_REG_DELAY_OFF 0
void cpu_write_reg(uint8_t reg, uint32_t val, uint8_t delay);

#endif
