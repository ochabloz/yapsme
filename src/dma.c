
#include "dma.h"
#include "memory_map.h"
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
        dma_state->DMA_control_register = 0x07654321;
        return DMA_SUCCESS;
    }
    return DMA_FAILURE;
}

uint32_t dma_desinit(void);

void dma_process_channel_control(uint32_t* channel_control, uint32_t value){
    *channel_control = value;
    uint32_t step = value & (0x01 << 1);
    uint32_t direction = value & (0x01 << 0);
    uint32_t base_addr = *(channel_control -2) & 0x00ffffff;
    uint32_t start_now = value & (0x01 << 24);
    uint32_t sync_mode = (value >> 9) & 0x3;

    if(start_now){
        // DMA transfert must start immediately
        *channel_control = value & ~(0x01 << 24); // clear start_now bit
        uint16_t transfert_size = *(channel_control -1);
        if(channel_control == &dma_state->OTC_channel_control){ // Clear the ordering Table
            for(uint32_t i = 0; i < (transfert_size-1); i++)
            {
                //printf("DMA transfert :0x%08x ->[0x%08x]\n", base_addr -4, base_addr);
                mm_write(base_addr, base_addr-4);
                base_addr -=4;
            }
            mm_write(base_addr, 0xffffff);
            //printf("DMA transfert :0x%08x ->[0x%08x]\n", 0xffffff, base_addr);
        }
        if(channel_control == &dma_state->GPU_channel_control){
            printf("base_addr = 0x%08x, sync_mode=%d, direction : %s\n", base_addr, sync_mode, (!direction) ? "+1" : "-1");
            uint32_t addr = mm_read(base_addr);
            uint8_t nb_bytes = addr >> 24;
            addr &= 0xffffff;
            printf("next element : 0x%06x, nb_bytes : %d\n", addr, nb_bytes);
            
            while(addr != 0xffffff){
                addr = mm_read(addr);
                nb_bytes = addr >> 24;
                addr &= 0xffffff;
                printf("next element : 0x%06x, nb_bytes : %d\n", addr, nb_bytes);
            }
            
        }
    }

}

uint32_t dma_read(uint8_t number, uint8_t addr){
    printf("dma read : %d, addr: %d\n", number, addr);
    switch (number){
        case 0:
            if (addr == 0x00) return dma_state->MDECin_base_addr;
            if (addr == 0x04) return dma_state->MDECin_block_control;
            if (addr == 0x08) return dma_state->MDECin_channel_control;
            break;
        case 1:
            if (addr == 0x00) return dma_state->MDECout_base_addr;
            if (addr == 0x04) return dma_state->MDECout_block_control;
            if (addr == 0x08) return dma_state->MDECout_channel_control;
            break;
        case 2:
            if (addr == 0x00) return dma_state->GPU_base_addr;
            if (addr == 0x04) return dma_state->GPU_block_control;
            if (addr == 0x08) return dma_state->GPU_channel_control;
            break;
        case 3:
            if (addr == 0x00) return dma_state->CDROM_base_addr;
            if (addr == 0x04) return dma_state->CDROM_block_control;
            if (addr == 0x08) return dma_state->CDROM_channel_control;
            break;
        case 4:
            if (addr == 0x00) return dma_state->SPU_base_addr;
            if (addr == 0x04) return dma_state->SPU_block_control;
            if (addr == 0x08) return dma_state->SPU_channel_control;
            break;
        case 5:
            if (addr == 0x00) return dma_state->PIO_base_addr;
            if (addr == 0x04) return dma_state->PIO_block_control;
            if (addr == 0x08) return dma_state->PIO_channel_control;
            break;
        case 6:
            if (addr == 0x00) return dma_state->OTC_base_addr;
            if (addr == 0x04) return dma_state->OTC_block_control;
            if (addr == 0x08) return dma_state->OTC_channel_control;
            break;
        case 7:
            if (addr == 0x00) return dma_state->DMA_control_register;
            if (addr == 0x04) return dma_state->DMA_interrupt_register;
            break;
        default:
            break;
    }
    return 0xFFFFFFFF;
}

void dma_write(uint8_t number, uint8_t addr, uint32_t value){
    printf("dma write : %d, addr: %d = 0x%08x\n", number, addr, value);
    switch (number)
    {
        case 0:
            if (addr == 0x00) {dma_state->MDECin_base_addr = value; break;}
            if (addr == 0x04) {dma_state->MDECin_block_control = value; break;}
            if (addr == 0x08) {dma_state->MDECin_channel_control = value; break;}
            break;
        case 1:
            if (addr == 0x00) {dma_state->MDECout_base_addr = value; break;}
            if (addr == 0x04) {dma_state->MDECout_block_control = value; break;}
            if (addr == 0x08) {dma_state->MDECout_channel_control = value; break;}
            break;
        case 2:
            if (addr == 0x00) {dma_state->GPU_base_addr = value; break;}
            if (addr == 0x04) {dma_state->GPU_block_control = value; break;}
            if (addr == 0x08) {dma_process_channel_control(&dma_state->GPU_channel_control, value); break;}
            break;
        case 3:
            if (addr == 0x00) {dma_state->CDROM_base_addr = value; break;}
            if (addr == 0x04) {dma_state->CDROM_block_control = value; break;}
            if (addr == 0x08) {dma_state->CDROM_channel_control = value; break;}
            break;
        case 4:
            if (addr == 0x00) {dma_state->SPU_base_addr = value; break;}
            if (addr == 0x04) {dma_state->SPU_block_control = value; break;}
            if (addr == 0x08) {dma_state->SPU_channel_control = value; break;}
            break;
        case 5:
            if (addr == 0x00) {dma_state->PIO_base_addr = value; break;}
            if (addr == 0x04) {dma_state->PIO_block_control = value; break;}
            if (addr == 0x08) {dma_state->PIO_channel_control = value; break;}
            break;
        case 6:
            if (addr == 0x00) {dma_state->OTC_base_addr = value; break;}
            if (addr == 0x04) {dma_state->OTC_block_control = value; break;}
            if (addr == 0x08) {dma_process_channel_control(&dma_state->OTC_channel_control, value); break;}
            break;
        case 7:
            if (addr == 0x00) {dma_state->DMA_control_register = value; break;}
            if (addr == 0x04) {dma_state->DMA_interrupt_register = value; break;}
            break;
        default:
            break;
    }
}