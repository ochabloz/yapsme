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
#include "spu.h"
#include <stdlib.h>
#include <stdio.h>
struct spu_struct{ // SPU ADDR range 0x1F801C00 - 0x1F801FFC
    
    uint32_t control_registers[64]; // 0x1F801D80 - 0x1F801DBC
};

typedef struct spu_struct * spu_state_t;

spu_state_t spu_state = NULL;

uint32_t * _get_register_pointer(uint32_t addr){
    if (spu_state == NULL){
        return NULL;
    }
    if (addr >= 0x180 && addr < (0x180 + 64)){
        return &(spu_state->control_registers[addr - 0x180]);
    }
    return NULL;
}

uint32_t spu_init(void){
    if (spu_state == NULL) {
        spu_state = malloc(sizeof(struct spu_struct));
        return SPU_SUCCESS;
    }
    return SPU_FAILURE;
}


void spu_register_write(uint32_t addr, uint32_t value){
    uint32_t * reg = _get_register_pointer(addr);
    if(reg != NULL){
        *reg = value;
    }
    else{
        printf("SPU register WRITE 0x%08x => (0x%04x) unimplemented !!\n", addr+ 0x1f801C00, addr);
    }
}

uint32_t spu_register_read(uint32_t addr){
    uint32_t * reg = _get_register_pointer(addr);
    if(reg != NULL){
        return *reg;
    }
    printf("SPU register READ  0x%08x => (0x%04x) unimplemented !!\n", addr+ 0x1f801C00, addr);
    return 0xFFFFFFFF;
}
