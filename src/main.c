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
#include "cpu.h"
#include "memory_map.h"
#include "spu.h"
#include "dma.h"
#include <dirent.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char const *argv[])
{

    struct dirent *bios_file;
    DIR *bios_dir = opendir("bios/");
    if (bios_dir == NULL)
    {
        printf("bios directory could not be opened\n");
        return 1;
    }
    int bios_loaded = 0;
    mm_initialise();
    spu_init();
    dma_initialise();
    while ((bios_file = readdir(bios_dir)) != NULL)
    {
        char filepath[265];
        if (strlen(bios_file->d_name) <= 80)
        {
            sprintf(filepath, "bios/%s", bios_file->d_name);
            if (mm_load_bios(filepath) == MM_SUCCESS)
            {
                bios_loaded = 1;
                printf("BIOS file %s loaded.\n", bios_file->d_name);
                break;
            }
        }
    }
    if (!bios_loaded)
    {
        printf("No bios file were found. Exiting..\n");
        return 1;
    }
    cpu_initialise();
    uint32_t i = 0;
    while (1)
    {
        if (cpu_run(i++) == CPU_FAILURE)
        {
            printf("cpu ran for %d cycles\n", i);
            break;
        }
        // if (i == 22858300)
        //     break;
    }
    return 0;
}
