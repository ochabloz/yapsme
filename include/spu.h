

#ifndef SPU_H
#define SPU_H
#include <stdint.h>

#define SPU_SUCCESS 0x00
#define SPU_FAILURE 0x01

uint32_t spu_init(void);
void spu_register_write(uint32_t addr, uint32_t value);
uint32_t spu_register_read(uint32_t addr);

#endif
