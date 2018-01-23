
#include "memory_map.h"
#include <stdio.h>
#include <stdlib.h>

#define MM_BIOS_SIZE 0x80000 // 512 KB


struct mm_state{
    uint32_t bios[MM_BIOS_SIZE / 4];
};

typedef struct mm_state * mm_state_t;

mm_state_t mm_state_live = NULL;

uint32_t mm_initialise(void){
    if (mm_state_live == NULL) {
        mm_state_live = malloc(sizeof(struct mm_state));
        return MM_SUCCESS;
    }
    return MM_FAILURE;
}

uint32_t mm_load_bios(const char * path){
    if(mm_state_live == NULL){
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
    fread(mm_state_live->bios, 4, MM_BIOS_SIZE / 4, bios_file);
    fclose(bios_file);

    return MM_SUCCESS;
}
