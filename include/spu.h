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

#ifndef SPU_H
#define SPU_H
#include <stdint.h>

#define SPU_SUCCESS 0x00
#define SPU_FAILURE 0x01

uint32_t spu_init(void);
void spu_register_write(uint32_t addr, uint32_t value);
uint32_t spu_register_read(uint32_t addr);

#endif
