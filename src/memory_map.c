// Copyright 2018 Olivier CHABLOZ
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#include "memory_map.h"
#include "spu.h"
#include "dma.h"
#include <stdio.h>
#include <stdlib.h>

#define MM_BIOS_SIZE 0x80000 // 512 KB
#define MM_RAM_SIZE  0x200000 // 2048 KB


struct mm_struct{
    uint32_t bios[MM_BIOS_SIZE / 4];
    uint32_t ram[MM_RAM_SIZE / 4];

    // memory control
    uint32_t bios_rom_size;
    uint32_t ram_size;
    uint32_t common_delay;
    uint32_t cache_control;
    uint32_t cache_isolated;
};

typedef struct mm_struct * mm_state_t;

mm_state_t mm_state = NULL;

uint32_t mm_initialise(void){
    if (mm_state == NULL) {
        mm_state = malloc(sizeof(struct mm_struct));
        mm_state->cache_isolated = 0;
        return MM_SUCCESS;
    }
    return MM_FAILURE;
}

uint32_t mm_desinit(void){
    if(mm_state == NULL){
        return MM_SUCCESS;
    }
    free(mm_state);
    mm_state = NULL;
    return MM_SUCCESS;
}

uint32_t mm_load_bios(const char * path){
    if(mm_state == NULL){
        return MM_FAILURE;
    }
    FILE *bios_file = fopen(path, "r");
    if(bios_file == NULL){
        return MM_FAILURE;
    }
    fseek(bios_file, 0, SEEK_END);
    if(ftell(bios_file) != MM_BIOS_SIZE){
        return MM_FAILURE;
    }
    rewind(bios_file);
    fread(mm_state->bios, 4, MM_BIOS_SIZE / 4, bios_file);
    fclose(bios_file);

    return MM_SUCCESS;
}

uint32_t mm_read(uint32_t addr){
    if(addr >= 0x00000000 && addr < 0x00000000 + MM_RAM_SIZE){ // KUSEG access
        return mm_state->ram[(addr - 0x00000000)/ 4];
    }
    if(addr >= 0x1f000000 && addr < 0x1f000000 + 0x100){    // expansion 1
        return 0xffffffff; // the return value when nothing is connected to the expansion slot
    }
    if (addr == 0x1F801070) return cpu_get_interrupt_status();
    if (addr == 0x1F801074) return cpu_get_interrupt_mask();
    if(addr >= 0x1F801080 && addr < 0x1F801100){    // DMA
        return dma_read((addr - 0x1F801080) >> 4, addr & 0xF);
    }
    if(addr == 0x1F801810 || addr == 0x1F801814){    // GPU
        if (addr == 0x1F801814) return 0x10000000;
        return 0x00000000;
    }
    if(addr >= 0x1f801C00 && addr < 0x1f802000){    // SPU
        return spu_register_read(addr - 0x1f801C00);
    }
	if(addr >= 0x80000000 && addr < 0x80000000 + MM_RAM_SIZE) { // KSEG0 access
		return mm_state->ram[(addr - 0x80000000) / 4];
	}
    if(addr >= 0xa0000000 && addr < 0xa0000000 + MM_RAM_SIZE){ // KSEG1 access
        return mm_state->ram[(addr & 0x1FFFFFFF)/ 4];
    }
    if(addr >= 0xbfc00000 && addr < 0xbfc00000 + MM_BIOS_SIZE){
        return mm_state->bios[(addr - 0xbfc00000)/ 4];
    }
    printf("mm_PANIC read to 0x%08x not implemented\n", addr);
    return 0xffffffff;
}

void cache(uint32_t addr, uint32_t instruction){
    //printf("cache write !\n");
    return;
}


void mm_set_status(uint32_t status_name, uint32_t status){
    switch (status_name) {
        case MM_S_ISOLATE_CACHE:
            mm_state->cache_isolated = status ? 1 : 0;
            //printf("cache is %s isolated !\n", status ? "" : "NOT");
            break;
        default:
            break;
    }
}


void mm_write(uint32_t addr, uint32_t data){
    if(addr >= 0x00000000 && addr < 0x00000000 + MM_RAM_SIZE){ // KUSEG
        if((mm_state->cache_isolated) && addr < 0x1000){ // code cache enabled. discard writes
            cache(addr, data);
        }
        else{
            mm_state->ram[(addr - 0x00000000)/ 4] = data;
        }
    }
    else if(addr >= 0x1f000000 && addr < 0x1f000000 + 0x100){    // expansion 1
        /*expansion 1 is not connected*/
    }
    else if(addr >= 0x1f800100 && addr < 0x1f800000 + 0x80000){ // IO map
        switch (addr){
            case 0x1f801010:
                mm_state->bios_rom_size = data;
                break;
            case 0x1f801060:
                mm_state->ram_size = data;
                break;
            case 0x1f801020:
                mm_state->common_delay = data;
                break;
            default:
            {
                if(addr >= 0x1f801C00 && addr < 0x1f802000){    // SPU
                    spu_register_write(addr - 0x1f801C00, data);
                }else{
                    printf("IO write [0x%08x] = 0x%08x not implemented\n", addr, data);
                }
            }
            break;
        }
    }
	else if (addr >= 0x80000000 && addr < 0x80000000 + MM_RAM_SIZE) { // KSEG0
		mm_state->ram[(addr - 0x80000000) / 4] = data;
	}
    else if(addr >= 0xa0000000 && addr < 0xa0000000 + MM_RAM_SIZE){ // KSEG1
        mm_state->ram[(addr & 0x1FFFFFFF)/ 4] = data;
    }
    else if(addr >= 0xFFFE0000 && addr < 0xFFFE0000 + 512){
        switch (addr) {
            case 0xfffe0130:
                mm_state->cache_control = data;
                break;
            default:
                printf("IO cache control write [0x%08x] = 0x%08x not implemented\n", addr, data);
                break;
        }
    }
    else{
        printf("mm_PANIC write [0x%08x] = 0x%08x not implemented\n", addr, data);
    }
    return;
}
