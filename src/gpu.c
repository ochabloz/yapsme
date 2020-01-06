#include "gpu.h"
#include <stdio.h>
#include <stdlib.h>

#define FIFO_SIZE 64

struct gpu_struct
{
    uint32_t enabled;
    uint32_t dma_direction;
    uint32_t reverse_flag;
    uint32_t display_mode;
    uint32_t horizontal_res2;

    uint32_t fifo[FIFO_SIZE];
};

typedef struct gpu_struct *gpu_state_t;

gpu_state_t gpu_state = NULL;

uint32_t gpu_initialise(void)
{
    if (gpu_state == NULL)
    {
        gpu_state = malloc(sizeof(struct gpu_struct));
        gpu_state->enabled = 0;
        return 1;
    }
    return 0;
}

void gpu_gp0_write(uint32_t value)
{
    uint8_t op = value >> 24;
    uint32_t param = value & 0xFFFFFF;
    printf("GPU: GP0 Received CMD: (0x%02x, 0x%06x) \n", op, param);
}

void gpu_gp1_write(uint32_t value)
{
    uint8_t op = value >> 24;
    uint32_t param = value & 0xFFFFFF;
    if (op == 0x00)
    {
        printf("GPU: RESET\n");
    }
    else if (op == 0x01)
    { // RESET FIFO
        for (size_t i = 0; i < FIFO_SIZE; i++)
        {
            gpu_state->fifo[i] = 0;
        }
    }
    else if (op == 0x02)
    {
        printf("GPU: Ack IRQ\n");
    }
    else if (op == 0x03)
    {
        //printf("GPU: Display is %s\n", param ? "off" : "on");
        gpu_state->enabled = !(param & 0x1);
    }
    else if (op == 0x04)
    {
        // 0-1  DMA Direction (0=Off, 1=FIFO, 2=CPUtoGP0, 3=GPUREADtoCPU)
        //printf("GPU: DMA direction %d\n", param);
        gpu_state->dma_direction = param & 0x3;
    }
    else if (op == 0x05)
    {
        uint16_t x = param & 0x3ff;
        uint16_t y = (param & (0x1ff << 10)) >> 10;
        printf("GPU: start of display area (%d, %d)\n", x, y);
    }
    else if (op == 0x06)
    {
        printf("GPU: Horizontal display range: %d\n", param);
    }
    else if (op == 0x07)
    {
        printf("GPU: Vertical display range: %d\n", param);
    }
    else if (op == 0x08)
    {
        printf("GPU: Display mode: %d\n", param);
        gpu_state->reverse_flag = (param & (0x1 << 7)) >> 7;
        gpu_state->horizontal_res2 = (param & (0x1 << 6)) >> 6;
        gpu_state->display_mode = param & 0x3f;
    }

    else
    {
        printf("GPU: GP1 Received CMD: (0x%02x, 0x%06x) \n", op, param);
    }
}

uint32_t gpu_read_gpustat()
{
    /*
    1F801814h - GPUSTAT - GPU Status Register (R)

    0-3   Texture page X Base   (N*64)                              ;GP0(E1h).0-3
    4     Texture page Y Base   (N*256) (ie. 0 or 256)              ;GP0(E1h).4
    5-6   Semi Transparency     (0=B/2+F/2, 1=B+F, 2=B-F, 3=B+F/4)  ;GP0(E1h).5-6
    7-8   Texture page colors   (0=4bit, 1=8bit, 2=15bit, 3=Reserved)GP0(E1h).7-8
    9     Dither 24bit to 15bit (0=Off/strip LSBs, 1=Dither Enabled);GP0(E1h).9
    10    Drawing to display area (0=Prohibited, 1=Allowed)         ;GP0(E1h).10
    11    Set Mask-bit when drawing pixels (0=No, 1=Yes/Mask)       ;GP0(E6h).0
    12    Draw Pixels           (0=Always, 1=Not to Masked areas)   ;GP0(E6h).1
    13    Interlace Field       (or, always 1 when GP1(08h).5=0)
    14    "Reverseflag"         (0=Normal, 1=Distorted)             ;GP1(08h).7
    15    Texture Disable       (0=Normal, 1=Disable Textures)      ;GP0(E1h).11
    16    Horizontal Resolution 2     (0=256/320/512/640, 1=368)    ;GP1(08h).6
    17-18 Horizontal Resolution 1     (0=256, 1=320, 2=512, 3=640)  ;GP1(08h).0-1
    19    Vertical Resolution         (0=240, 1=480, when Bit22=1)  ;GP1(08h).2
    20    Video Mode                  (0=NTSC/60Hz, 1=PAL/50Hz)     ;GP1(08h).3
    21    Display Area Color Depth    (0=15bit, 1=24bit)            ;GP1(08h).4
    22    Vertical Interlace          (0=Off, 1=On)                 ;GP1(08h).5
    23    Display Enable              (0=Enabled, 1=Disabled)       ;GP1(03h).0
    24    Interrupt Request (IRQ1)    (0=Off, 1=IRQ)       ;GP0(1Fh)/GP1(02h)
    25    DMA / Data Request, meaning depends on GP1(04h) DMA Direction:
            When GP1(04h)=0 ---> Always zero (0)
            When GP1(04h)=1 ---> FIFO State  (0=Full, 1=Not Full)
            When GP1(04h)=2 ---> Same as GPUSTAT.28
            When GP1(04h)=3 ---> Same as GPUSTAT.27
    26    Ready to receive Cmd Word   (0=No, 1=Ready)  ;GP0(...) ;via GP0
    27    Ready to send VRAM to CPU   (0=No, 1=Ready)  ;GP0(C0h) ;via GPUREAD
    28    Ready to receive DMA Block  (0=No, 1=Ready)  ;GP0(...) ;via GP0
    29-30 DMA Direction (0=Off, 1=?, 2=CPUtoGP0, 3=GPUREADtoCPU)    ;GP1(04h).0-1
    31    Drawing even/odd lines in interlace mode (0=Even or Vblank, 1=Odd)
*/
    uint32_t gpustat = 0x1c000000;
    gpustat = gpustat | gpu_state->dma_direction << 29;
    gpustat = gpustat | !(gpu_state->enabled << 23);
    printf("GPU: Read GPUSTAT 0x%08x\n", gpustat);
    return gpustat;
}

uint32_t gpu_read_gpuread()
{
    printf("GPU: Read GPUREAD 0x%08x\n", 0);
    return 0x00000000;
}
