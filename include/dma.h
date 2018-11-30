
#ifndef DMA_H
#define DMA_H

#include <stdint.h>

uint32_t dma_initialise(void);
uint32_t dma_desinit(void);

uint32_t dma_read(uint8_t number, uint8_t addr);
void dma_write(uint8_t number, uint8_t addr, uint32_t value);

#define DMA_SUCCESS 0x00
#define DMA_FAILURE 0x01

#endif