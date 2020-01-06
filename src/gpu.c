#include "gpu.h"
#include <stdio.h>

void gpu_gp0_write(uint32_t value){
    uint8_t op = value >> 24;
    uint32_t param = value & 0xFFFFFF;
    printf("GPU: GP0 Received CMD: (0x%02x, 0x%06x) \n", op, param);
}

void gpu_gp1_write(uint32_t value){
    uint8_t op = value >> 24;
    uint32_t param = value & 0xFFFFFF;
    printf("GPU: GP1 Received CMD: (0x%02x, 0x%06x) \n", op, param);
}

uint32_t gpu_read_gpustat(){
    uint32_t gpustat = 0x1c000000;
    printf("GPU: Read GPUSTAT 0x%08x\n", gpustat);
    return gpustat;
}

uint32_t gpu_read_gpuread(){
    printf("GPU: Read GPUSTAT 0x%08x\n", 0);
    return 0x00000000;
}
