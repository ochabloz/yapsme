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
    uint32_t LOW;
    uint32_t PC_delay;

    uint8_t  R_delay_reg;  // register delay slot
    uint32_t R_delay_dat;
    uint8_t  R_delay_flag; // Status machine for the delay register
    uint8_t  R_delay_cpu;

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
        cpu_state->R_delay_flag = CPU_REG_DELAY__NO_DATA_AVAILABLE;
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
void print_disassemble(uint32_t instruction);

uint32_t cpu_run(uint32_t nb_cycles){
    uint32_t fetch = mm_read(cpu_state->PC);
    if (nb_cycles > ~0){ // 71500
        print_disassemble(fetch);
    }
    return cpu_execute(fetch);

}


uint32_t cp0_execute(uint32_t instruction);
void cp0_instruction_mtc(uint8_t r_target, uint8_t r_dest);
void cp0_instruction_mfc(uint8_t r_target, uint8_t r_dest);

// The whole instruction set
void cpu_instruction_lui(uint8_t r_target, uint16_t immediate);
void cpu_instruction_ori(uint8_t r_source, uint8_t r_target, uint16_t immediate);
void cpu_instruction_andi(uint8_t r_source, uint8_t r_target, uint16_t immediate);
void cpu_instruction_or(uint8_t r_target, uint8_t r_dest, uint8_t r_source);
void cpu_instruction_and(uint8_t r_target, uint8_t r_dest, uint8_t r_source);
void cpu_instruction_sltu(uint8_t r_source, uint8_t r_target, uint8_t r_dest);
void cpu_instruction_addu(uint8_t r_source, uint8_t r_target, uint8_t r_dest);
void cpu_instruction_add(uint8_t r_source, uint8_t r_target, uint8_t r_dest);

void cpu_instruction_sw(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_sh(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_sb(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_lw(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_lb(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_lbu(uint8_t r_source, uint8_t r_target, int16_t immediate);
void cpu_instruction_sll(uint8_t r_target, uint8_t r_dest, uint8_t shift_amount);
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
    uint32_t return_status = CPU_SUCCESS;


    if(opcode == 0){
        switch (instruction & 0x3F) {
            case 0x00:
                cpu_instruction_sll(rt, rd, imm5);
                break;
            case 0x08:
                cpu_instruction_jr(rs);
                break;
            case 0x09:
                cpu_instruction_jalr(rs, rd);
                break;
            case 0x20:
                cpu_instruction_add(rs, rt, rd);
                break;
            case 0x21:
                cpu_instruction_addu(rs, rt, rd);
                break;
            case 0x24:
                cpu_instruction_and(rt, rd, rs);
                //printf("PC : 0x%08x, instruction: 0x%08x\n", cpu_state->PC, instruction);
                break;
            case 0x25:
                cpu_instruction_or(rt, rd, rs);
                break;
            case 0x2b:
                cpu_instruction_sltu(rs, rt, rd);
                break;

            default:
            {
                printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",instruction, cpu_state->PC);
                printf("OP 0x%02x, rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, sec_op 0x%02x\n",
                opcode, rs, rt, rd, imm5, instruction & 0x3F);
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
            case 0x23:
                cpu_instruction_lw(rs, rt, imm16);
                break;
            case 0x24:
                cpu_instruction_lbu(rs, rt, imm16);
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
                        printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",instruction, cpu_state->PC);
                        printf("OP 0x%02x, rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, imm16 0x%04x\n",
                        opcode, rs, rt, rd, imm5, imm16);
                            return_status = CPU_FAILURE;
                            break;
                    }

                }
                else{
                    printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",instruction, cpu_state->PC);
                    printf("OP 0x%02x, rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, imm16 0x%04x\n",
                    opcode, rs, rt, rd, imm5, imm16);
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
    uint32_t return_status = CPU_SUCCESS;

    switch (cp0_opcode) {
        case 0x00:
            cp0_instruction_mfc(rt, rd);
            break;
        case 0x04:
            cp0_instruction_mtc(rt, rd);
            break;
        default:
            printf("instruction 0x%08x not implemented at PC = 0x%08x!!\n",instruction, cpu_state->PC);
            printf("CP0 0x%02x instruction not defined rt(0x%02x), rd(0x%02x) compl_opcode(0x%02x)\n",
                    cp0_opcode, rt, rd, instruction & 0x3F);
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
        if(reg == 12){
            mm_set_status(MM_S_ISOLATE_CACHE, val & (0x1 << 16));
        }
    }
}

/** CPU Instructions **/

// load 16bits to the upper part of the target register
void cpu_instruction_lui(uint8_t r_target, uint16_t immediate){
    cpu_state->regs[r_target] = immediate << 16;
}

// Store Word signed
void cpu_instruction_sw(uint8_t r_source, uint8_t r_target, int16_t immediate){
    mm_write(cpu_state->regs[r_source] + immediate, cpu_state->regs[r_target]);
}

void cpu_instruction_sh(uint8_t r_source, uint8_t r_target, int16_t immediate){
    int addr = immediate + cpu_state->regs[r_source];
    uint32_t data = mm_read(addr & ~(0x3));
    if(addr % 4){
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
    uint32_t val = mm_read(cpu_state->regs[r_source] + immediate);
    cpu_write_reg(r_target, val, CPU_REG_DELAY_ON);
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

// add immediate unsigned
void cpu_instruction_addiu(uint8_t r_source, uint8_t r_target, int16_t immediate){
    cpu_state->regs[r_target] = cpu_state->regs[r_source] + immediate;
}

// add immediate signed
void cpu_instruction_addi(uint8_t r_source, uint8_t r_target, int16_t immediate){
    cpu_state->regs[r_target] = cpu_state->regs[r_source] + immediate;
    // NOTE : ADD overflow
}

// Jump
void cpu_instruction_j(uint32_t jump){
    cpu_state->PC_delay = (cpu_state->PC & (0xF0000000)) | jump;
}

// Jump and link
// store return address to RA (register 31)
void cpu_instruction_jal(uint32_t jump){
    //printf("JAL: register r[%d] => 0x%08x\n",31, cpu_state->PC);
    cpu_state->regs[31] = cpu_state->PC;
    cpu_state->PC_delay = (cpu_state->PC & (0xF0000000)) | jump;
}

// Jump register
void cpu_instruction_jr(uint8_t r_source){
    //printf("jump register back to r[%d] => 0x%08x\n",r_source, cpu_state->regs[r_source]);
    cpu_state->PC_delay = cpu_state->regs[r_source];
}

void cpu_instruction_jalr(uint8_t r_source, uint8_t r_dest){
    cpu_state->regs[r_dest] = cpu_state->PC;
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
        cpu_state->PC_delay = (test >= 0) ? branch : 0;
    }
    else{
        cpu_state->PC_delay = (test < 0) ? branch : 0;
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

void cpu_instruction_sltu(uint8_t r_source, uint8_t r_target, uint8_t r_dest){
    cpu_state->regs[r_dest] = (cpu_state->regs[r_source] < cpu_state->regs[r_target]) ? 1 : 0;
}

void cpu_instruction_addu(uint8_t r_source, uint8_t r_target, uint8_t r_dest){
    cpu_state->regs[r_dest] = cpu_state->regs[r_source] + cpu_state->regs[r_target];
}

void cpu_instruction_add(uint8_t r_source, uint8_t r_target, uint8_t r_dest){
    uint32_t r_tmp = cpu_state->regs[r_source];
    cpu_state->regs[r_dest] = cpu_state->regs[r_source] + cpu_state->regs[r_target];
    if(r_tmp > cpu_state->regs[r_dest]){
        printf("implement overflow!\n");
    }
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




void print_disassemble(uint32_t instruction){
    printf("0x%08x : ", cpu_state->PC);
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
                printf("sec_op 0x%02x, rt 0x%02x, imm5 0x%02x\n", sec_op, rt, imm5);
                break;
            case 0x08:
                printf("jr, $%d            # 0x%08x\n", rs, cpu_state->regs[rs]);
                break;
            default:
                printf("sec_op 0x%02x, $%d = $%d, $%d\n", sec_op, rd, rs, rt);
                break;
        }
    }
    else{
        switch (opcode) {
            case 0x04:
            case 0x05:
                printf("%s $%d, $%d, 0x%08x\n", (opcode == 4) ? "beq" : "bne",rt, rs, cpu_state->PC + 4 + (((int16_t)imm16) << 2));
                break;
            case 0x02:
            case 0x03:
                printf("%s, 0x%08x\n", (opcode == 2) ? "j" : "jal",(cpu_state->PC & 0xF0000000) | imm26 << 2);
                break;
            case 0x08:
            case 0x09:
                printf("%s $%d, $%d, %d        # 0x%08x + 0x%04x\n", (opcode == 8) ? "addi" : "addiu" , rs, rt, (int16_t)imm16, cpu_state->regs[rs], imm16);
                break;
            case 0x0d:
                printf("ori, $%d, $%d,  0x%04x        # 0x%08x | 0x%04x\n", rs, rt, imm16, cpu_state->regs[rs], imm16);
                break;
            case 0x0c:
            case 0x20:
            case 0x23:
            case 0x28:
            case 0x29:
                printf("OP 0x%02x, rs 0x%02x, rt 0x%02x, imm16 %d\n", opcode, rs, rt, (int16_t)imm16);
                break;
            case 0x2b:
                printf("sw [$%d + 0x%04x], $%d    # 0x%08x = 0x%08x\n",rs, imm16, rt, cpu_state->regs[rs] + (int16_t)imm16, cpu_state->regs[rt]);
                break;
            case 0x0f:
                printf("lui $%d, 0x%04x        # 0x%08x\n", rt, imm16, imm16 << 16);
                break;
            default:
                printf("OP 0x%02x, rs  0x%02x, rt  0x%02x, rd  0x%02x, imm5  0x%02x, imm16 0x%04x\n",
                opcode, rs, rt, rd, imm5, imm16);
                break;
        }
    }
}
