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
#include <stdlib.h>
#include <stdio.h>


#define CPU_REG_DELAY__DATA_READY_TO_BE_FETCHED 1
#define CPU_REG_DELAY__DATA_IN_SKIP_SYNC 2
#define CPU_REG_DELAY__NO_DATA_AVAILABLE 3
#define CPU_REG_DELAY__CP0 0
#define CPU_REG_DELAY__CP2 2
#define CPU_REG_DELAY__CPU 4

struct cpu_struct {
    uint32_t regs [32];
    uint32_t PC;
    uint32_t HI;
    uint32_t LO;
    uint32_t PC_delay;
    uint8_t  Branch_delay; // 1 when executing an instruction during a branch delay slot. Else 0

    uint8_t  R_delay_reg;  // register delay slot
    uint32_t R_delay_dat;
    uint8_t  R_delay_flag; // Status machine for the delay register
    uint8_t  R_delay_cpu;

    uint32_t cp0_regs[64];
    uint32_t interrupt_status;
    uint32_t interrupt_mask;
};

typedef struct cpu_struct * cpu_state_t;

cpu_state_t cpu_state = NULL;

uint32_t cpu_initialise(void){
    if (cpu_state == NULL) {
        cpu_state = malloc(sizeof(struct cpu_struct));
        cpu_state->PC = 0xbfc00000;
        cpu_state->PC_delay = ~0;
        cpu_state->regs[0x00] = 0;
        cpu_state->R_delay_flag = CPU_REG_DELAY__NO_DATA_AVAILABLE;
        cpu_state->Branch_delay = 0;
        cpu_state->interrupt_mask = 0;
        cpu_state->interrupt_status = 0;
        return CPU_SUCCESS;
    }
    return CPU_FAILURE;
}

uint32_t cpu_desinit(void){
    if(cpu_state == NULL){
        return CPU_SUCCESS;
    }
    free(cpu_state);
    cpu_state = NULL;
    return CPU_SUCCESS;
}

enum exception{
    addr_error_load = 0x04,
    addr_error_store,
    bus_error_instr_fetch,
    bus_error_data_ls,
    system_call,
    breakpoint,
    reserved_instr,
    coprocessor_unusable,
    math_overflow
};
typedef enum exception exception_t;
void cpu_trigger_exception(exception_t cause);



void print_disassemble(uint32_t instruction);

uint32_t cpu_run(uint32_t nb_cycles){
    if(cpu_state->PC % 4){
        cpu_trigger_exception(bus_error_instr_fetch);
        return CPU_SUCCESS;
    }
    else{
        uint32_t fetch = mm_read(cpu_state->PC);
        if (nb_cycles > 12848300){ // 2857421
            print_disassemble(fetch);
        }
        return cpu_execute(fetch);
    }
}

uint32_t cpu_get_interrupt_status(){
    return cpu_state->interrupt_status;
}
uint32_t cpu_get_interrupt_mask(){
    return cpu_state->interrupt_mask;
}
void cpu_set_interrupt_mask(uint32_t mask){
    cpu_state->interrupt_mask = mask;
}

uint32_t cp0_execute(uint32_t instruction);
void cp0_instruction_mtc(uint8_t r_target, uint8_t r_dest);
void cp0_instruction_mfc(uint8_t r_target, uint8_t r_dest);
void cp0_instruction_rfe(uint8_t compl_opcode);

// The whole instruction set
void cpu_instruction_syscall(void);
void cpu_instruction_lui(uint8_t r_target, uint16_t immediate);
void cpu_instruction_ori(uint8_t r_source, uint8_t r_target, uint16_t immediate);
void cpu_instruction_andi(uint8_t r_source, uint8_t r_target, uint16_t immediate);
void cpu_instruction_or(uint8_t r_target, uint8_t r_dest, uint8_t r_source);
void cpu_instruction_xor(uint8_t r_target, uint8_t r_dest, uint8_t r_source);
void cpu_instruction_nor(uint8_t r_target, uint8_t r_dest, uint8_t r_source);
void cpu_instruction_and(uint8_t r_target, uint8_t r_dest, uint8_t r_source);
void cpu_instruction_sltu(uint8_t r_source, uint8_t r_target, uint8_t r_dest);
void cpu_instruction_slt(uint8_t r_source, uint8_t r_target, uint8_t r_dest);
void cpu_instruction_slti(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_sltiu(uint8_t r_source, uint8_t r_target, uint16_t immediate);
void cpu_instruction_addu(uint8_t r_source, uint8_t r_target, uint8_t r_dest);
void cpu_instruction_subu(uint8_t r_source, uint8_t r_target, uint8_t r_dest);
void cpu_instruction_add(uint8_t r_source, uint8_t r_target, uint8_t r_dest);

void cpu_instruction_div(uint8_t r_source, uint8_t r_target);
void cpu_instruction_divu(uint8_t r_source, uint8_t r_target);
void cpu_instruction_mflo(uint8_t r_dest);
void cpu_instruction_mtlo(uint8_t r_source);
void cpu_instruction_mfhi(uint8_t r_dest);
void cpu_instruction_mthi(uint8_t r_source);

void cpu_instruction_sw(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_sh(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_sb(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_lw(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_lhu(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_lh(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_lb(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_lbu(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_sll(uint8_t r_target, uint8_t r_dest, uint8_t shift_amount);
void cpu_instruction_sllv(uint8_t r_target, uint8_t r_dest, uint8_t r_source);
void cpu_instruction_sra(uint8_t r_target, uint8_t r_dest, uint8_t shift_amount);
void cpu_instruction_srl(uint8_t r_target, uint8_t r_dest, uint8_t shift_amount);
void cpu_instruction_addi(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_addiu(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_j(uint32_t jump);
void cpu_instruction_jal(uint32_t jump);
void cpu_instruction_jalr(uint8_t r_source, uint8_t r_dest);
void cpu_instruction_jr(uint8_t r_source);
void cpu_instruction_beq(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_bne(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_bxx(uint8_t r_source, uint8_t r_target, int16_t immediate);

void cpu_instruction_bgtz(uint8_t r_source, int16_t immediate);
void cpu_instruction_blez(uint8_t r_source, int16_t immediate);


uint32_t cpu_execute(uint32_t instruction){

    // When a jump or a branch occurs, the next instruction is exectuted, then the jump occurs
    if(cpu_state->PC_delay != ~0x00){
        cpu_state->PC = cpu_state->PC_delay;
        cpu_state->Branch_delay = 1;
        cpu_state->PC_delay = ~0x00;
    }
    else{
        cpu_state->PC += 4;
        cpu_state->Branch_delay = 0;
    }

    // instruction decoding :
    uint8_t opcode = instruction >> 26;
    uint8_t rs = (instruction >> 21) & 0x1F;
    uint8_t rt = (instruction >> 16) & 0x1F;
    uint8_t rd = (instruction >> 11) & 0x1F;
    uint8_t imm5 = (instruction >> 6) & 0x1F;
    uint16_t imm16 = (uint16_t)instruction;
    uint32_t imm26 = instruction & 0x03FFFFFF;
    uint32_t return_status = CPU_SUCCESS;


    if(opcode == 0){
        switch (instruction & 0x3F) {
            case 0x00:
                cpu_instruction_sll(rt, rd, imm5);
                break;
            case 0x02:
                cpu_instruction_srl(rt, rd, imm5);
                break;
            case 0x03:
                cpu_instruction_sra(rt, rd, imm5);
                break;
            case 0x04:
                cpu_instruction_sllv(rt, rd, rs);
                break;
            case 0x08:
                cpu_instruction_jr(rs);
                break;
            case 0x09:
                cpu_instruction_jalr(rs, rd);
                break;
            case 0x0c:
                cpu_instruction_syscall();
                break;
            case 0x10:
                cpu_instruction_mfhi(rd);
                break;
            case 0x11:
                cpu_instruction_mthi(rs);
                break;
            case 0x12:
                cpu_instruction_mflo(rd);
                break;
            case 0x13:
                cpu_instruction_mtlo(rs);
                break;
            case 0x1a:
                cpu_instruction_div(rs, rt);
                break;
            case 0x1b:
                cpu_instruction_divu(rs, rt);
                break;
            case 0x20:
                cpu_instruction_add(rs, rt, rd);
                break;
            case 0x21:
                cpu_instruction_addu(rs, rt, rd);
                break;
            case 0x23:
                cpu_instruction_subu(rs, rt, rd);
                break;
            case 0x24:
                cpu_instruction_and(rt, rd, rs);
                //printf("PC : 0x%08x, instruction: 0x%08x\n", cpu_state->PC, instruction);
                break;
            case 0x25:
                cpu_instruction_or(rt, rd, rs);
                break;
            case 0x26:
                cpu_instruction_xor(rt, rd, rs);
                break;
            case 0x27:
                cpu_instruction_nor(rt, rd, rs);
                //printf("PC : 0x%08x, instruction: 0x%08x\n", cpu_state->PC, instruction);
                break;
            case 0x2a:
                cpu_instruction_slt(rs, rt, rd);
                break;
            case 0x2b:
                cpu_instruction_sltu(rs, rt, rd);
                break;

            default:
            {
                printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",
                                                                    instruction, cpu_state->PC);
                printf("OP 0x%02x, ", opcode);
                printf("rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, sec_op 0x%02x\n",
                rs, rt, rd, imm5, instruction & 0x3F);
                return_status = CPU_FAILURE;
                break;
            }
        }
    }
    else{
        switch (opcode) {
            case 0x01:
                cpu_instruction_bxx(rs, rt, imm16);
                break;
            case 0x02:
                cpu_instruction_j(imm26 << 2);
                break;
            case 0x03:
                cpu_instruction_jal(imm26 << 2);
                break;
            case 0x04:
                cpu_instruction_beq(rs, rt, imm16);
                break;
            case 0x05:
                cpu_instruction_bne(rs, rt, imm16);
                break;
            case 0x06:
                cpu_instruction_blez(rs, imm16);
                break;
            case 0x07:
                cpu_instruction_bgtz(rs, imm16);
                break;
            case 0x08:
                cpu_instruction_addi(rs, rt, imm16);
                break;
            case 0x09:
                cpu_instruction_addiu(rs, rt, imm16);
                break;
            case 0x0A:
                cpu_instruction_slti(rs, rt, imm16);
                break;
            case 0x0B:
                cpu_instruction_sltiu(rs, rt, imm16);
                break;
            case 0x0C:
                cpu_instruction_andi(rs, rt, imm16);
                break;
            case 0x0D:
                cpu_instruction_ori(rs, rt, imm16);
                break;
            case 0x0F: // LUI  (Load Upper Immediate)
                cpu_instruction_lui(rt, imm16);
                break;
            case 0x20:
                cpu_instruction_lb(rs, rt, imm16);
                break;
            case 0x21:
                cpu_instruction_lhu(rs, rt, imm16);
                break;
            case 0x23:
                cpu_instruction_lw(rs, rt, imm16);
                break;
            case 0x24:
                cpu_instruction_lbu(rs, rt, imm16);
                break;
            case 0x25:
                cpu_instruction_lhu(rs, rt, imm16);
                break;
            case 0x28:
                cpu_instruction_sb(rs, rt, imm16);
                break;
            case 0x29:
                cpu_instruction_sh(rs, rt, imm16);
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
                            return_status = cp0_execute(instruction);
                            break;
                        default:
                        printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",
                                                                    instruction, cpu_state->PC);
                        printf("OP 0x%02x, ", opcode);
                        printf("rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, imm16 0x%04x\n",
                        rs, rt, rd, imm5, imm16);
                            return_status = CPU_FAILURE;
                            break;
                    }

                }
                else{
                    printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",
                    instruction, cpu_state->PC);
                    printf("OP 0x%02x, ", opcode);
                    printf("rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, imm16 0x%04x\n",
                    rs, rt, rd, imm5, imm16);
                    return_status = CPU_FAILURE;
                }
            }
        }
    }
    if(cpu_state->R_delay_flag == CPU_REG_DELAY__DATA_IN_SKIP_SYNC){
        cpu_state->R_delay_flag = CPU_REG_DELAY__DATA_READY_TO_BE_FETCHED;
    }
    else if(cpu_state->R_delay_flag == CPU_REG_DELAY__DATA_READY_TO_BE_FETCHED){
        switch (cpu_state->R_delay_cpu) {
            case CPU_REG_DELAY__CPU:
                cpu_state->regs[cpu_state->R_delay_reg] = cpu_state->R_delay_dat;
                break;
            case CPU_REG_DELAY__CP0:
                cpu_state->cp0_regs[cpu_state->R_delay_reg] = cpu_state->R_delay_dat;
                break;
        }

        cpu_state->R_delay_flag = CPU_REG_DELAY__NO_DATA_AVAILABLE;
    }

    // update status at the end of execution
    cpu_state->regs[0] = 0;
    return return_status;
}

/* COPROCESSOR 0*/
uint32_t cp0_execute(uint32_t instruction){
    // decode instruction
    instruction &= 0x03FFFFFF;
    uint8_t cp0_opcode = instruction >> 21;
    uint8_t rt = (instruction >> 16) & 0x1F;
    uint8_t rd = (instruction >> 11) & 0x1F;
    uint8_t compl_opcode = instruction & 0x3F;
    uint32_t return_status = CPU_SUCCESS;

    switch (cp0_opcode) {
        case 0x00:
            cp0_instruction_mfc(rt, rd);
            break;
        case 0x04:
            cp0_instruction_mtc(rt, rd);
            break;
        case 0x10:
            cp0_instruction_rfe(compl_opcode);
            break;
        default:
            printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",
                                                            instruction, cpu_state->PC);
            printf("CP0 0x%02x instruction not defined ", cp0_opcode);
            printf("rt(0x%02x), rd(0x%02x) compl_opcode(0x%02x)\n",
                    rt, rd, compl_opcode);
            return_status = CPU_FAILURE;
            break;
    }
    return return_status;
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
        return cpu_state->LO;
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
            cpu_state->R_delay_cpu = CPU_REG_DELAY__CPU;
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
        cpu_state->LO = val;
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
        if(reg == 12){
            mm_set_status(MM_S_ISOLATE_CACHE, val & (0x1 << 16));
        }
    }
}

void cpu_trigger_exception(exception_t cause){
    // the jump address depends on the BEV field of the (sr) cop0_12 register (bit 22)
    uint32_t jump_address = (cpu_state->cp0_regs[12] & (0x1 << 22)) != 0 ?  0xbfc00180 : 0x80000080;

    uint8_t mode = cpu_state->cp0_regs[12] & 0x3f;
    cpu_state->cp0_regs[12] &= ~0x3f; // clear mode
    cpu_state->cp0_regs[12] |= (mode << 2) & 0x3f;

    // CAUSE register
    cpu_state->cp0_regs[13] = cause << 2;
    // EPC (exception program counter) register
    cpu_state->cp0_regs[14] = cpu_state->PC;
    cpu_state->PC = jump_address;

    if(cpu_state->Branch_delay){
        cpu_state->cp0_regs[13] |= 0x1 << 31;
        // the exception has occured during a branch delai. must compare execution flow with actual
        // hardware or emulator
        printf("Exception occured during branch delai!!! at PC 0x%08x\n",cpu_state->cp0_regs[14]);
    }
}

/** CPU Instructions **/

void cpu_instruction_syscall(void){
    cpu_trigger_exception(system_call);
}

// load 16bits to the upper part of the target register
void cpu_instruction_lui(uint8_t r_target, uint16_t immediate){
    cpu_state->regs[r_target] = immediate << 16;
}

// Store Word signed
void cpu_instruction_sw(uint8_t r_source, uint8_t r_target, int16_t immediate){
    uint32_t addr = immediate + cpu_state->regs[r_source];
    if(addr %4){
        cpu_trigger_exception(addr_error_store);
        return;
    }
    mm_write(addr, cpu_state->regs[r_target]);
}

void cpu_instruction_sh(uint8_t r_source, uint8_t r_target, int16_t immediate){
    uint32_t addr = immediate + cpu_state->regs[r_source];
    if(addr %2){
        cpu_trigger_exception(addr_error_store);
        return;
    }
    uint32_t data = mm_read(addr & ~(0x3));
    if((addr % 4)){
        data = (data & 0xFFFF) | (cpu_state->regs[r_target] << 16);
        
    }
    else{
        data = (data & 0xFFFF0000) | cpu_state->regs[r_target];
    }
    mm_write(addr, data);
}

void cpu_instruction_sb(uint8_t r_source, uint8_t r_target, int16_t immediate){
    int addr = immediate + cpu_state->regs[r_source];
    uint32_t data = mm_read(addr & ~(0x3));
    uint32_t mask = ~(0xff << (8 * (addr % 4)));
    data = (data & mask) | (cpu_state->regs[r_target] << (8 * (addr % 4)));
    mm_write(addr, data);
}

// load word : 32 bits loaded from memory to the target register (rt = mem[rs + imm16])
void cpu_instruction_lw(uint8_t r_source, uint8_t r_target, int16_t immediate){
    uint32_t addr = cpu_state->regs[r_source] + immediate;
    if(addr %4){
        cpu_trigger_exception(addr_error_load);
        return;
    }
    uint32_t val = mm_read(addr);
    cpu_write_reg(r_target, val, CPU_REG_DELAY_ON);
}

// load half word unsigned. The 2 MSB are filled with 0's
void cpu_instruction_lhu(uint8_t r_source, uint8_t r_target, int16_t immediate){
    uint32_t addr = cpu_state->regs[r_source] + immediate;
    if(addr %2){
        cpu_trigger_exception(addr_error_load);
        return;
    }
    uint32_t val = (mm_read(addr & ~0x3) >> (addr %4)) & 0xFF;
    cpu_write_reg(r_target, val, CPU_REG_DELAY_ON);
}

// load half word signed. The 2 MSB are filled with 1's
void cpu_instruction_lh(uint8_t r_source, uint8_t r_target, int16_t immediate){
    uint32_t addr = cpu_state->regs[r_source] + immediate;
    if(addr %2){
        cpu_trigger_exception(addr_error_load);
        return;
    }
    int32_t val = (mm_read(addr & ~0x3) >> (addr %4)) & 0xFF;
    cpu_write_reg(r_target, (val<< 16)>>16, CPU_REG_DELAY_ON);
}

// load Byte
void cpu_instruction_lb(uint8_t r_source, uint8_t r_target, int16_t immediate){
    uint32_t addr = (cpu_state->regs[r_source] + immediate);
    int32_t val = (mm_read(addr & ~0x3) >> (8* (addr % 4))) & 0xFF;
    cpu_write_reg(r_target, (val << 24) >> 24, CPU_REG_DELAY_ON);
}

// load Byte unsigned
void cpu_instruction_lbu(uint8_t r_source, uint8_t r_target, int16_t immediate){
    uint32_t addr = (cpu_state->regs[r_source] + immediate);
    uint32_t val = (mm_read(addr & ~0x3) >> (8* (addr % 4))) & 0xFF;
    cpu_write_reg(r_target, val, CPU_REG_DELAY_ON);
}

// shift logical left
void cpu_instruction_sll(uint8_t r_target, uint8_t r_dest, uint8_t shift_amount){
    cpu_state->regs[r_dest] = cpu_state->regs[r_target] << shift_amount;
}

// shift logical left variable. The shift amount is stored in a register.
void cpu_instruction_sllv(uint8_t r_target, uint8_t r_dest, uint8_t r_source){
    // mips keeps only the 5 LSb for the shift amount
    cpu_state->regs[r_dest] = cpu_state->regs[r_target] <<  (cpu_state->regs[r_source] & 0x1f);
}

// shift logical right
void cpu_instruction_srl(uint8_t r_target, uint8_t r_dest, uint8_t shift_amount){
    cpu_state->regs[r_dest] = cpu_state->regs[r_target] >> shift_amount;
}

// shift right arithmetic
void cpu_instruction_sra(uint8_t r_target, uint8_t r_dest, uint8_t shift_amount){
    int32_t val = cpu_state->regs[r_target]; // register is casted to be signed
    cpu_state->regs[r_dest] = val >> shift_amount;
}

// add immediate unsigned
void cpu_instruction_addiu(uint8_t r_source, uint8_t r_target, int16_t immediate){
    cpu_state->regs[r_target] = cpu_state->regs[r_source] + immediate;
}

// add immediate signed
void cpu_instruction_addi(uint8_t r_source, uint8_t r_target, int16_t immediate){
    int32_t s = cpu_state->regs[r_source];
    int16_t i = immediate;
    int32_t r = cpu_state->regs[r_source] + immediate;
    cpu_state->regs[r_target] = r;
    if(s < 0 && i < 0 && r >= 0){
        cpu_trigger_exception(math_overflow);
    }
    else if(s > 0 && i > 0 && r <= 0){
        cpu_trigger_exception(math_overflow);
    }
}

// Jump
void cpu_instruction_j(uint32_t jump){
    cpu_state->PC_delay = (cpu_state->PC & (0xF0000000)) | jump;
}

// Jump and link
// store return address to RA (register 31)
void cpu_instruction_jal(uint32_t jump){
    // store pc + 4 since the next instruction will be executed before the jump
    cpu_state->regs[31] = cpu_state->PC + 4;
    cpu_state->PC_delay = (cpu_state->PC & (0xF0000000)) | jump;
}

// Jump register
void cpu_instruction_jr(uint8_t r_source){
    //printf("jump register back to r[%d] => 0x%08x\n",r_source, cpu_state->regs[r_source]);
    cpu_state->PC_delay = cpu_state->regs[r_source];
}

void cpu_instruction_jalr(uint8_t r_source, uint8_t r_dest){
    // store pc + 4 since the next instruction will be executed before the jump
    cpu_state->regs[r_dest] = cpu_state->PC + 4;
    cpu_state->PC_delay = cpu_state->regs[r_source];
}

// branch not equal
void cpu_instruction_bne(uint8_t r_source, uint8_t r_target, int16_t immediate){
    if(cpu_state->regs[r_target] != cpu_state->regs[r_source]){
        cpu_state->PC_delay = cpu_state->PC + (immediate << 2);
    }
}

// branch equal
void cpu_instruction_beq(uint8_t r_source, uint8_t r_target, int16_t immediate){
    if(cpu_state->regs[r_target] == cpu_state->regs[r_source]){
        cpu_state->PC_delay = cpu_state->PC + (immediate << 2);
    }
}

// branch lower or equal than zero
void cpu_instruction_blez(uint8_t r_source, int16_t immediate){
    if(((int32_t)(cpu_state->regs[r_source])) <= 0){
        cpu_state->PC_delay = cpu_state->PC + (immediate << 2);
    }
}

// branch greater than zero
void cpu_instruction_bgtz(uint8_t r_source, int16_t immediate){
    if(((int32_t)(cpu_state->regs[r_source])) > 0){
        cpu_state->PC_delay = cpu_state->PC + (immediate << 2);
    }
}

// BLTZ, BGEZ, BLTZAL, BGEZAL
void cpu_instruction_bxx(uint8_t r_source, uint8_t r_target, int16_t immediate){
    int32_t test = cpu_state->regs[r_source];
    int32_t branch = cpu_state->PC + (immediate << 2);
    // LSb == 1 : branch if Greater or equal than 0
    // LSb == 0 : branch if lower than 0
    if(r_target & 0x1){
        cpu_state->PC_delay = (test >= 0) ? branch : ~0;
    }
    else{
        cpu_state->PC_delay = (test < 0) ? branch : ~0;
    }
    // MSb == 1 : And link to
    if(r_target & 0x10){ // pc is stored to RA even if no branching
        cpu_state->regs[31] = cpu_state->PC;
    }
}

// OR
void cpu_instruction_or(uint8_t r_target, uint8_t r_dest, uint8_t r_source){
    cpu_state->regs[r_dest] = cpu_state->regs[r_source] | cpu_state->regs[r_target];
}

// XOR
void cpu_instruction_xor(uint8_t r_target, uint8_t r_dest, uint8_t r_source){
    cpu_state->regs[r_dest] = cpu_state->regs[r_source] ^ cpu_state->regs[r_target];
}

// NOR
void cpu_instruction_nor(uint8_t r_target, uint8_t r_dest, uint8_t r_source){
    cpu_state->regs[r_dest] = ~(cpu_state->regs[r_source] | cpu_state->regs[r_target]);
}

// AND
void cpu_instruction_and(uint8_t r_target, uint8_t r_dest, uint8_t r_source){
    cpu_state->regs[r_dest] = cpu_state->regs[r_source] & cpu_state->regs[r_target];
}

void cpu_instruction_ori(uint8_t r_source, uint8_t r_target, uint16_t immediate){
    cpu_state->regs[r_target] = cpu_state->regs[r_source] | immediate;
}

void cpu_instruction_andi(uint8_t r_source, uint8_t r_target, uint16_t immediate){
    cpu_state->regs[r_target] = cpu_state->regs[r_source] & immediate;
}

// set if lower than (signed)
void cpu_instruction_slt(uint8_t r_source, uint8_t r_target, uint8_t r_dest){
    int32_t test = cpu_state->regs[r_source];
    cpu_state->regs[r_dest] = (test < ((int32_t)cpu_state->regs[r_target])) ? 1 : 0;
}

// set if lower than (unsigned)
void cpu_instruction_sltu(uint8_t r_source, uint8_t r_target, uint8_t r_dest){
    cpu_state->regs[r_dest] = (cpu_state->regs[r_source] < cpu_state->regs[r_target]) ? 1 : 0;
}

// set if lower than immediate (signed)
void cpu_instruction_slti(uint8_t r_source, uint8_t r_target, int16_t immediate){
    cpu_state->regs[r_target] = (((int32_t)cpu_state->regs[r_source]) < immediate) ? 1 : 0;
}

// set if lower than immediate (unsigned)
void cpu_instruction_sltiu(uint8_t r_source, uint8_t r_target, uint16_t immediate){
    cpu_state->regs[r_target] = (cpu_state->regs[r_source] < immediate) ? 1 : 0;
}

void cpu_instruction_addu(uint8_t r_source, uint8_t r_target, uint8_t r_dest){
    cpu_state->regs[r_dest] = cpu_state->regs[r_source] + cpu_state->regs[r_target];
}

void cpu_instruction_subu(uint8_t r_source, uint8_t r_target, uint8_t r_dest){
    cpu_state->regs[r_dest] = cpu_state->regs[r_source] - cpu_state->regs[r_target];
}

void cpu_instruction_add(uint8_t r_source, uint8_t r_target, uint8_t r_dest){
    int32_t s = cpu_state->regs[r_source];
    int16_t t =  cpu_state->regs[r_target];
    int32_t r = s + t;
    cpu_state->regs[r_dest] = r;
    if(s < 0 && t < 0 && r >= 0){
        cpu_trigger_exception(math_overflow);
    }
    else if(s > 0 && t > 0 && r <= 0){
        cpu_trigger_exception(math_overflow);
    }
}


void cpu_instruction_div(uint8_t r_source, uint8_t r_target){
    int32_t numerator = cpu_state->regs[r_source];
    int32_t denominator = cpu_state->regs[r_target];
    if(denominator == 0){ // division by 0 will give bogus results
        cpu_state->LO = (numerator >= 0) ? -1 : 1;
    }
    else if (numerator == 0x80000000 && denominator == 0xffffffff){
        cpu_state->LO = 0x80000000; // another special case
        cpu_state->HI = 0;
    }
    else{
        cpu_state->LO = numerator / denominator;
        cpu_state->HI = numerator % denominator;
    }
}

void cpu_instruction_divu(uint8_t r_source, uint8_t r_target){
    uint32_t numerator = cpu_state->regs[r_source];
    uint32_t denominator = cpu_state->regs[r_target];
    if(denominator == 0){ // division by 0 will give bogus results
        cpu_state->LO = 0xffffffff; // another special case
        cpu_state->HI = numerator;
    }
    else{
        cpu_state->LO = numerator / denominator;
        cpu_state->HI = numerator % denominator;
    }
}

void cpu_instruction_mflo(uint8_t r_dest){
    cpu_state->regs[r_dest] = cpu_state->LO;
}

void cpu_instruction_mfhi(uint8_t r_dest){
    cpu_state->regs[r_dest] = cpu_state->HI;
}

void cpu_instruction_mtlo(uint8_t r_source){
    cpu_state->LO = cpu_state->regs[r_source];
}

void cpu_instruction_mthi(uint8_t r_source){
    cpu_state->HI = cpu_state->regs[r_source];
}


/** COPROCESSOR 0 Instructions **/
// Move to coprocessor 0
void cp0_instruction_mtc(uint8_t r_target, uint8_t r_dest){
    cp0_write_reg(r_dest, cpu_state->regs[r_target]);
}

// Move from coprocessor 0
void cp0_instruction_mfc(uint8_t r_target, uint8_t r_dest){
    cpu_write_reg(r_target, cpu_state->cp0_regs[r_dest], CPU_REG_DELAY_ON);
}

void cp0_instruction_rfe(uint8_t compl_opcode){
    if(compl_opcode != 0x10){
        printf("RFE instruction with compl_opcode 0x%02x is not implemented!!\n", compl_opcode);
        // NOTE: trigger exception..
    }

    uint8_t mode = cpu_state->cp0_regs[12] & 0x3f;
    cpu_state->cp0_regs[12] &= ~0x3f; // clear mode
    cpu_state->cp0_regs[12] |= (mode >> 2) & 0x3f;
}



void print_disassemble(uint32_t instruction){
    printf("0x%08x : 0x%08x\t(ra: 0x%08x)\t", cpu_state->PC, instruction, cpu_state->regs[31]);
    if(instruction == 0x00){
        printf("nop\n");
        return;
    }
    // instruction decoding :
    uint8_t opcode = instruction >> 26;
    uint8_t rs = (instruction >> 21) & 0x1F;
    uint8_t rt = (instruction >> 16) & 0x1F;
    uint8_t rd = (instruction >> 11) & 0x1F;
    uint8_t imm5 = (instruction >> 6) & 0x1F;
    uint16_t imm16 = (uint16_t)instruction;
    uint32_t imm26 = instruction & 0x03FFFFFF;
    uint8_t sec_op = instruction & 0x3F;

    if (opcode == 0) {
        switch (sec_op) {
            case 0x00:
                printf("sll $%02d, $%02d, 0x%02x\t\t\t# 0x%08x = 0x%08x << 0x%02x\n", rd, rt, 
                    imm5, cpu_state->regs[rt] << imm5, cpu_state->regs[rt], imm5);
                break;
            case 0x08:
                printf("jr, $%d\t\t\t# 0x%08x\n", rs, cpu_state->regs[rs]);
                break;
            case 0x24:
                printf("and, $%02d, $%02d, $%02d\t\t\t# 0x%08x & 0x%08x\n", rd, rs, rt,
                         cpu_state->regs[rs], cpu_state->regs[rt]);
                break;
            case 0x25:
                printf("or, $%02d, $%02d, $%02d\t\t\t# 0x%08x | 0x%08x\n", rd, rs, rt,
                         cpu_state->regs[rs], cpu_state->regs[rt]);
                break;
            
            case 0x27:
                printf("nor, $%02d, $%02d, $%02d\t\t\t# 0x%08x | 0x%08x\n", rd, rs, rt,
                         cpu_state->regs[rs], cpu_state->regs[rt]);
                break;

            default:
                printf("sec_op 0x%02x, $%02d = $%02d, $%02d\n", sec_op, rd, rs, rt);
                break;
        }
    }
    else{
        switch (opcode) {
            case 0x04:
            case 0x05:
                printf("%s $%02d, $%02d, 0x%08x\n", (opcode == 4) ? "beq" : "bne",
                                            rt, rs, cpu_state->PC + 4 + (((int16_t)imm16) << 2));
                break;
            case 0x02:
            case 0x03:
                printf("%s, 0x%08x\n", (opcode == 2) ? "j" : "jal",
                                                    (cpu_state->PC & 0xF0000000) | imm26 << 2);
                break;
            case 0x07:
                printf("bgtz $%02d, %04d\t\t\t# 0x%08x\n",rs, (int16_t)imm16<<2,
                                                    cpu_state->PC + (imm16<<2));
                break;
            case 0x08:
            case 0x09:
                printf("%s $%02d, $%02d, %d\t\t\t# 0x%08x + 0x%04x\n",
                                        (opcode == 8) ? "addi" : "addiu" , rs, rt,
                                        (int16_t)imm16, cpu_state->regs[rs], imm16);
                break;
            case 0x0d:
                printf("ori, $%d, $%d,  0x%04x\t\t# 0x%08x | 0x%04x\n", rs, rt, imm16,
                                                    cpu_state->regs[rs], imm16);
                break;
            case 0x23:
                printf("lw $%02d, [$%02d + 0x%05x]\t\t# 0x%08x<-[0x%08x]\n", rt, rs, imm16, 
                    mm_read(cpu_state->regs[rs] + imm16), cpu_state->regs[rs] + imm16);
                break;
            case 0x24:
                printf("lbu, $%d, [$%d + %d]\t\t# 0x%08x = [0x%08x]\n",rt, rs,(int16_t)imm16,
                                            cpu_state->regs[rt], cpu_state->regs[rs] + (imm16));
                break;
            case 0x25:
                printf("lhu, $%d, [$%d + %d]\t# $%d <- [0x%08x]\n",rt, rs, (int16_t)imm16,
                                            rt, cpu_state->regs[rs] + (imm16));
                break;
            case 0x28:
                printf("sb [$%d + %d], $%d\t\t\t# [0x%08x] = 0x%08x\n", rs,(int16_t)imm16,
                                        rt, cpu_state->regs[rs] + (imm16), cpu_state->regs[rt]);
                break;
            case 0x29:
                printf("sh [$%02d + %02d], $%02d\t\t# [0x%08x] = 0x%08x\n", rs,(int16_t)imm16,
                                        rt, cpu_state->regs[rs] + (imm16), cpu_state->regs[rt]);
                break;
            case 0x0c:
            case 0x20:
                printf("OP 0x%02x, rs 0x%02x, rt 0x%02x, imm16 %d\n",opcode,rs, rt,(int16_t)imm16);
                break;
            case 0x2b:
                printf("sw [$%d + 0x%04x], $%d\t\t# [0x%08x]<- 0x%08x\n",rs, imm16, rt,
                                        cpu_state->regs[rs] + (int16_t)imm16, cpu_state->regs[rt]);
                break;
            case 0x0f:
                printf("lui $%02d, 0x%04x\t\t\t# $%d <- 0x%08x\n", rt, imm16, rt, imm16 << 16);
                break;
            default:
                printf("OP 0x%02x, ", opcode);
                printf("rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, imm16 0x%04x\n",
                rs, rt, rd, imm5, imm16);
                break;
        }
    }
}
