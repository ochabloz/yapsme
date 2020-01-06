#ifndef GPU_H
#define GPU_H

#include <stdint.h>

void gpu_gp0_write(uint32_t value);
void gpu_gp1_write(uint32_t value);
uint32_t gpu_read_gpustat();
uint32_t gpu_read_gpuread();

#endif