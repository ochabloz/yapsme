
#include "memory_map.h"
#include <stdio.h>
#include <stdlib.h>

#define MM_BIOS_SIZE 0x80000 // 512 KB
#define MM_RAM_SIZE  0x200000 // 2048 KB


struct mm_struct{
    uint32_t bios[MM_BIOS_SIZE / 4];
    uint32_t ram[MM_RAM_SIZE / 4];
};

typedef struct mm_struct * mm_state_t;

mm_state_t mm_state = NULL;

uint32_t mm_initialise(void){
    if (mm_state == NULL) {
        mm_state = malloc(sizeof(struct mm_struct));
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
    if(addr >= 0xa0000000 && addr < 0xa0000000 + MM_RAM_SIZE){
        return mm_state->ram[(addr - 0xa0000000)/ 4];
    }
    if(addr >= 0xbfc00000 && addr < 0xbfc00000 + MM_BIOS_SIZE){
        return mm_state->bios[(addr - 0xbfc00000)/ 4];
    }
    printf("mm_PANIC read to 0x%08x not implemented\n", addr);
    return 0xff;
}


void mm_write(uint32_t addr, uint32_t data){
    if(addr >= 0xa0000000 && addr < 0xa0000000 + MM_RAM_SIZE){
        mm_state->ram[(addr - 0xa0000000)/ 4] = data;
    }
    else{
        //printf("mm_PANIC write [0x%08x] = 0x%08x not implemented\n", addr, data);
    }
    return;
}
