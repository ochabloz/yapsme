#include "cpu.h"
#include "memory_map.h"
#include <stdlib.h>
#include <stdio.h>

struct cpu_struct {
    uint32_t regs [32];
    uint32_t PC;
    uint32_t HI;
    uint32_t LOW;
};

typedef struct cpu_struct * cpu_state_t;

cpu_state_t cpu_state = NULL;

uint32_t cpu_initialise(void){
    if (cpu_state == NULL) {
        cpu_state = malloc(sizeof(struct cpu_struct));
        cpu_state->PC = 0xbfc00000;
        cpu_state->regs[0x00] = 0;
        return MM_SUCCESS;
    }
    return MM_FAILURE;
}

uint32_t cpu_desinit(void){
    if(cpu_state == NULL){
        return CPU_SUCCESS;
    }
    free(cpu_state);
    cpu_state = NULL;
    return CPU_SUCCESS;
}

void cpu_run(uint32_t nb_cycles){
    uint32_t fetch = mm_read(cpu_state->PC);
    cpu_state->PC += 4;
    cpu_execute(fetch);
}


void cpu_instruction_lui(uint8_t r_target, uint16_t immediate);
void cpu_instruction_ori(uint8_t r_source, uint8_t r_target, uint16_t immediate);
void cpu_instruction_store_32bits(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_sll(uint8_t r_target, uint8_t r_dest, uint8_t shift_amount);
void cpu_instruction_addiu(uint8_t r_source, uint8_t r_target, int16_t immediate);

void cpu_execute(uint32_t instruction){
    // instruction decoding :
    uint8_t opcode, rt, rs, rd, imm5;
    uint16_t imm16;
    uint32_t imm26;
    opcode = instruction >> 26;
    rs = (instruction >> 21) & 0x1F;
    rt = (instruction >> 16) & 0x1F;
    rd = (instruction >> 11) & 0x1F;
    imm5 = (instruction >> 6) & 0x1F;
    imm16 = (uint16_t)instruction;
    imm26 = instruction & 0x03FFFFFF;


    if(opcode == 0b000000){
        switch (instruction & 0x1F) {
            case 0x00:
                cpu_instruction_sll(rt, rd, imm5);
                break;

            default:
            {
                printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",instruction, cpu_state->PC);
                printf("OP 0x%02x, rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, imm16 0x%04x\n",
                opcode, rs, rt, rd, imm5, imm16);
                break;
            }
        }
    }
    else{
        switch (opcode) {
            case 0x09:
                cpu_instruction_addiu(rs, rt, imm16);
                break;
            case 0x0F: // LUI  (Load Upper Immediate)
                cpu_instruction_lui(rt, imm16);
                break;
            case 0x0D:
                cpu_instruction_ori(rs, rt, imm16);
                break;
            case 0x2B:
                cpu_instruction_store_32bits(rs, rt, imm16);
                break;

            default:
            {
                printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",instruction, cpu_state->PC);
                printf("OP 0x%02x, rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, imm16 0x%04x\n",
                opcode, rs, rt, rd, imm5, imm16);
                if (opcode & 0b010000){
                    // coprocessor instruction
                }
            }
        }
    }
    cpu_state->regs[0] = 0;
    return;
}

uint32_t cpu_read_reg(uint8_t reg){
    if(reg < 32){
        return cpu_state->regs[reg];
    }
    if(reg == CPU_PC){
        return cpu_state->PC;
    }
    if(reg == CPU_HI){
        return cpu_state->HI;
    }
    if(reg == CPU_LO){
        return cpu_state->LOW;
    }
    return 0xFF;
}

void cpu_write_reg(uint8_t reg, uint32_t val){
    if(reg < 32){
        cpu_state->regs[reg] = val;
    }
    else if(reg == CPU_PC){
        cpu_state->PC = val;
    }
    else if(reg == CPU_HI){
        cpu_state->HI = val;
    }
    else if(reg == CPU_LO){
        cpu_state->LOW = val;
    }
}

// load 16bits to the upper part of the target register
void cpu_instruction_lui(uint8_t r_target, uint16_t immediate){
    cpu_state->regs[r_target] = immediate << 16;
}

void cpu_instruction_ori(uint8_t r_source, uint8_t r_target, uint16_t immediate){
    cpu_state->regs[r_target] = cpu_state->regs[r_source] | immediate;
}

void cpu_instruction_store_32bits(uint8_t r_source, uint8_t r_target, int16_t immediate){
    int imm = immediate << 16 >> 16;
    mm_write(imm + cpu_state->regs[r_source], cpu_state->regs[r_target]);
}

// shift logical left
void cpu_instruction_sll(uint8_t r_target, uint8_t r_dest, uint8_t shift_amount){
    cpu_state->regs[r_dest] = cpu_state->regs[r_target] << shift_amount;
}

// add immediate unsigned
void cpu_instruction_addiu(uint8_t r_source, uint8_t r_target, int16_t immediate){
    cpu_state->regs[r_target] = immediate + cpu_state->regs[r_source];
}
