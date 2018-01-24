#ifndef MEMORY_MAP_H
#define MEMORY_MAP_H

#include <stdint.h>

#define MM_SUCCESS   0x00
#define MM_FAILURE   0x01

uint32_t mm_initialise(void);
uint32_t mm_desinit(void);
uint32_t mm_load_bios(const char * path);
uint32_t mm_read(uint32_t addr);
void mm_write(uint32_t addr, uint32_t data);
//void mm_load_state(mm_state_t s);
//void mm_save_state(mm_state_t s);
#endif
