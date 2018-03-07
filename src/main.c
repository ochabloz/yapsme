
#include "cpu.h"
#include "memory_map.h"
#include "spu.h"
#include <dirent.h>
#include <stdio.h>
#include <string.h>


int main(int argc, char const *argv[]) {
    struct dirent * bios_file;
    DIR * bios_dir = opendir("bios/");
    if(bios_dir == NULL){
        printf("bios directory could not be opened\n");
        return 1;
    }
    int bios_loaded = 0;
    mm_initialise();
    spu_init();
    while ((bios_file = readdir(bios_dir)) != NULL) {
        char filepath[85];
        if(strlen(bios_file->d_name) <= 80){
            sprintf(filepath, "bios/%s", bios_file->d_name);
            if(mm_load_bios(filepath) == MM_SUCCESS){
                bios_loaded = 1;
                printf("BIOS file %s loaded.\n",bios_file->d_name);
                break;
            }
        }
    }
    if (!bios_loaded) {
        printf("No bios file were found. Exiting..\n");
        return 1;
    }
    cpu_initialise();
    uint32_t i = 0;
    while(1){
        if(cpu_run(i++) == CPU_FAILURE){
            printf("cpu ran for %d cycles\n", i);
            break;
        }
    }
    return 0;
}
