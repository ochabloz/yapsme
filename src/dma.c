
#include "dma.h"
#include <stdio.h>
#include <stdlib.h>

struct dma_struct{
    
    uint32_t MDECin_base_addr;
    uint32_t MDECin_block_control;
    uint32_t MDECin_channel_control;

    uint32_t MDECout_base_addr;
    uint32_t MDECout_block_control;
    uint32_t MDECout_channel_control;

    uint32_t GPU_base_addr;
    uint32_t GPU_block_control;
    uint32_t GPU_channel_control;

    uint32_t CDROM_base_addr;
    uint32_t CDROM_block_control;
    uint32_t CDROM_channel_control;

    uint32_t SPU_base_addr;
    uint32_t SPU_block_control;
    uint32_t SPU_channel_control;

    uint32_t PIO_base_addr;
    uint32_t PIO_block_control;
    uint32_t PIO_channel_control;

    uint32_t OTC_base_addr;
    uint32_t OTC_block_control;
    uint32_t OTC_channel_control;

    uint32_t DMA_control_register;
    uint32_t DMA_interrupt_register;
};

typedef struct dma_struct * dma_state_t;

dma_state_t dma_state = NULL;

uint32_t dma_initialise(void){
    if (dma_state == NULL) {
        dma_state = malloc(sizeof(struct dma_struct));
        dma_state->OTC_block_control = 0x0;
        return DMA_SUCCESS;
    }
    return DMA_FAILURE;
}

uint32_t dma_desinit(void);

uint32_t dma_read(uint8_t number, uint8_t addr){
    printf("dma read : %d, addr: %d\n", number, addr);
    switch (number){
        case 6:
            if (addr == 0x00) return dma_state->OTC_base_addr;
            if (addr == 0x04) return dma_state->OTC_block_control;
            if (addr == 0x08) return dma_state->OTC_channel_control;
            break;
    
        default:
            break;
    }
    return 0xFFFFFFFF;
}
void dma_write(uint8_t number, uint8_t addr, uint32_t value);