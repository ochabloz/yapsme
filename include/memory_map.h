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

#define MM_S_ISOLATE_CACHE 0x01
void mm_set_status(uint32_t status_name, uint32_t status);
//void mm_load_state(mm_state_t s);
//void mm_save_state(mm_state_t s);
#endif
