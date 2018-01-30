#include "cpu.h"
#include "memory_map.h"
#include <stdlib.h>
#include <stdio.h>


#define CPU_REG_DELAY__DATA_READY_TO_BE_FETCHED 1
#define CPU_REG_DELAY__DATA_IN_SKIP_SYNC 2
#define CPU_REG_DELAY__NO_DATA_AVAILABLE 3

struct cpu_struct {
    uint32_t regs [32];
    uint32_t PC;
    uint32_t HI;
    uint32_t LOW;
    uint32_t PC_delay;

    uint8_t R_delay_reg;  // register delay slot
    uint32_t R_delay_dat;
    uint8_t R_delay_flag; // Status machine for the delay register

    uint32_t cp0_regs[64];
};

typedef struct cpu_struct * cpu_state_t;

cpu_state_t cpu_state = NULL;

uint32_t cpu_initialise(void){
    if (cpu_state == NULL) {
        cpu_state = malloc(sizeof(struct cpu_struct));
        cpu_state->PC = 0xbfc00000;
        cpu_state->PC_delay = 0;
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
    cpu_execute(fetch);
}

void cp0_execute(uint32_t instruction);
void cp0_instruction_mtc(uint8_t r_target, uint8_t r_dest);

// The whole instruction set
void cpu_instruction_lui(uint8_t r_target, uint16_t immediate);
void cpu_instruction_ori(uint8_t r_source, uint8_t r_target, uint16_t immediate);
void cpu_instruction_or(uint8_t r_target, uint8_t r_dest, uint8_t r_source);
void cpu_instruction_sw(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_lw(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_sll(uint8_t r_target, uint8_t r_dest, uint8_t shift_amount);
void cpu_instruction_addi(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_addiu(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_j(uint32_t jump);
void cpu_instruction_bne(uint8_t r_source, uint8_t r_target, int16_t immediate);


void cpu_execute(uint32_t instruction){

    // When a jump or a branch occurs, the next instruction is exectuted, then the jump occurs
    cpu_state->PC = (cpu_state->PC_delay) ? cpu_state->PC_delay : cpu_state->PC +4;
    cpu_state->PC_delay = 0;

    // instruction decoding :
    uint8_t opcode = instruction >> 26;
    uint8_t rs = (instruction >> 21) & 0x1F;
    uint8_t rt = (instruction >> 16) & 0x1F;
    uint8_t rd = (instruction >> 11) & 0x1F;
    uint8_t imm5 = (instruction >> 6) & 0x1F;
    uint16_t imm16 = (uint16_t)instruction;
    uint32_t imm26 = instruction & 0x03FFFFFF;


    if(opcode == 0b000000){
        switch (instruction & 0x3F) {
            case 0x00:
                cpu_instruction_sll(rt, rd, imm5);
                break;
            case 0x25:
                cpu_instruction_or(rt, rd, rs);
                break;

            default:
            {
                printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",instruction, cpu_state->PC);
                printf("OP 0x%02x, rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, imm16 0x%04x, sec_op 0x%02x\n",
                opcode, rs, rt, rd, imm5, imm16, instruction & 0x3F);
                break;
            }
        }
    }
    else{
        switch (opcode) {
            case 0x02:
                cpu_instruction_j(imm26 << 2);
                break;
            case 0x05:
                cpu_instruction_bne(rs, rt, imm16);
                break;
            case 0x08:
                cpu_instruction_addi(rs, rt, imm16);
                break;
            case 0x09:
                cpu_instruction_addiu(rs, rt, imm16);
                break;
            case 0x0D:
                cpu_instruction_ori(rs, rt, imm16);
                break;
            case 0x0F: // LUI  (Load Upper Immediate)
                cpu_instruction_lui(rt, imm16);
                break;
            case 0x23:
                cpu_instruction_lw(rs, rt, imm16);
                break;
            case 0x2B:
                cpu_instruction_sw(rs, rt, imm16);
                break;

            default:
            {
                if (opcode & 0b010000){
                    // coprocessor instruction
                    switch (opcode & 0b000011) {
                        case 0:
                            cp0_execute(instruction);
                            break;
                        default:
                        printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",instruction, cpu_state->PC);
                        printf("OP 0x%02x, rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, imm16 0x%04x\n",
                        opcode, rs, rt, rd, imm5, imm16);
                            break;
                    }

                }
                else{
                    printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",instruction, cpu_state->PC);
                    printf("OP 0x%02x, rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, imm16 0x%04x\n",
                    opcode, rs, rt, rd, imm5, imm16);
                }
            }
        }
    }
    if(cpu_state->R_delay_flag == CPU_REG_DELAY__DATA_IN_SKIP_SYNC){
        cpu_state->R_delay_flag = CPU_REG_DELAY__DATA_READY_TO_BE_FETCHED;
    }
    else if(cpu_state->R_delay_flag == CPU_REG_DELAY__DATA_READY_TO_BE_FETCHED){
        cpu_state->regs[cpu_state->R_delay_reg] = cpu_state->R_delay_dat;
        cpu_state->R_delay_flag = CPU_REG_DELAY__NO_DATA_AVAILABLE;
    }

    // update status at the end of execution
    cpu_state->regs[0] = 0;
    return;
}

/* COPROCESSOR 0*/
void cp0_execute(uint32_t instruction){
    // decode instruction
    instruction &= 0x03FFFFFF;
    uint8_t cp0_opcode = instruction >> 21;
    uint8_t rt = (instruction >> 16) & 0x1F;
    uint8_t rd = (instruction >> 11) & 0x1F;

    switch (cp0_opcode) {
        case 0x04:
            cp0_instruction_mtc(rt, rd);
            break;
        default:
            printf("CP0 0x%02x instruction not defined rt(0x%02x), rd(0x%02x) compl_opcode(0x%02x)\n",
                    cp0_opcode, rt, rd, instruction & 0x3F);
            break;
    }
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

void cpu_write_reg(uint8_t reg, uint32_t val, uint8_t delay){
    if(reg < 32){
        if(cpu_state->R_delay_flag == CPU_REG_DELAY__DATA_READY_TO_BE_FETCHED){
            cpu_state->regs[cpu_state->R_delay_reg] = cpu_state->R_delay_dat;
            cpu_state->R_delay_flag = CPU_REG_DELAY__NO_DATA_AVAILABLE;
            // NOTE: In hardware it is likely that the cpu is paused until data from
            //        memory is fetched. Hence some sort of cycle counter should be added.
        }
        if(delay == CPU_REG_DELAY_ON){
            cpu_state->R_delay_reg = reg;
            cpu_state->R_delay_dat = val;
            cpu_state->R_delay_flag = CPU_REG_DELAY__DATA_IN_SKIP_SYNC;
        }
        else{
            cpu_state->regs[reg] = val;
        }
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
    else{
        printf("ERROR :: Forbidden register write. PC @ 0x%08x\n", cpu_state->PC);
    }
}

uint32_t cp0_read_reg(uint8_t reg){
    if(reg < 64){
        return cpu_state->cp0_regs[reg];
    }
    return 0xFF;
}

void cp0_write_reg(uint8_t reg, uint32_t val){
    if(reg < 64){
        cpu_state->cp0_regs[reg] = val;
    }
}

/** CPU Instructions **/

// load 16bits to the upper part of the target register
void cpu_instruction_lui(uint8_t r_target, uint16_t immediate){
    cpu_state->regs[r_target] = immediate << 16;
}

// Store Word signed
void cpu_instruction_sw(uint8_t r_source, uint8_t r_target, int16_t immediate){
    int imm = immediate << 16 >> 16;
    mm_write(imm + cpu_state->regs[r_source], cpu_state->regs[r_target]);
}

// load word : 32 bits loaded from memory to the target register (rt = mem[rs + imm16])
void cpu_instruction_lw(uint8_t r_source, uint8_t r_target, int16_t immediate){
    uint32_t val = mm_read(cpu_state->regs[r_source] + immediate);
    cpu_write_reg(r_target, val, CPU_REG_DELAY_ON);
}

// shift logical left
void cpu_instruction_sll(uint8_t r_target, uint8_t r_dest, uint8_t shift_amount){
    cpu_state->regs[r_dest] = cpu_state->regs[r_target] << shift_amount;
}

// add immediate unsigned
void cpu_instruction_addiu(uint8_t r_source, uint8_t r_target, int16_t immediate){
    cpu_state->regs[r_target] = immediate + cpu_state->regs[r_source];
}

// add immediate signed
void cpu_instruction_addi(uint8_t r_source, uint8_t r_target, int16_t immediate){
    cpu_state->regs[r_target] = cpu_state->regs[r_source] + (immediate << 16 >> 16);
    // NOTE : ADD overflow
}

// Jump
void cpu_instruction_j(uint32_t jump){
    cpu_state->PC_delay = (cpu_state->PC & (0xF0000000)) | jump;
}

// branch not equal
void cpu_instruction_bne(uint8_t r_source, uint8_t r_target, int16_t immediate){
    if(cpu_state->regs[r_target] != cpu_state->regs[r_source]){
        cpu_state->PC_delay = cpu_state->PC + (immediate << 16 >> 14) -4;
        // -4 since PC has already been incremented
    }
}

// OR
void cpu_instruction_or(uint8_t r_target, uint8_t r_dest, uint8_t r_source){
    cpu_state->regs[r_dest] = cpu_state->regs[r_source] | cpu_state->regs[r_target];
}

void cpu_instruction_ori(uint8_t r_source, uint8_t r_target, uint16_t immediate){
    cpu_state->regs[r_target] = cpu_state->regs[r_source] | immediate;
}




/** COPROCESSOR 0 Instructions **/
// Move to coprocessor 0
void cp0_instruction_mtc(uint8_t r_target, uint8_t r_dest){
    cpu_state->cp0_regs[r_dest] = cpu_state->regs[r_target];
}
