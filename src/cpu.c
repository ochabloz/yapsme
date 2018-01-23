#include "cpu.h"

struct cpu_struct {
    uint32_t regs [32];
    uint32_t PC;
    uint32_t HI;
    uint32_t LOW;
};
