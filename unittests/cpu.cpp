
#include <CppUTest/TestHarness.h>

extern "C"
{
#include "memory_map.h"
#include "cpu.h"
#include "unistd.h"
}

TEST_GROUP(cpu){};

TEST(cpu, initialisation){
    cpu_desinit();
    mm_initialise();
    mm_load_bios("bios/scph5502.bin");

    uint32_t rep = cpu_initialise();
    LONGS_EQUAL(0, rep);
    rep = cpu_read_reg(CPU_PC);
    LONGS_EQUAL(0xbfc00000, rep); // this is the PC default value
}


TEST_GROUP(cpu_instruction){
    void setup(){
        mm_initialise();
        mm_load_bios("bios/scph5502.bin");
        cpu_initialise();
    }

    void teardown(){
        cpu_desinit();
        mm_desinit();
    }
};

TEST(cpu_instruction, lui){
    cpu_execute(0x3c080013); // 1st instruction of the bios
    uint32_t rep = cpu_read_reg(0x8);
    LONGS_EQUAL(0x13 << 16, rep);

    cpu_execute(0xac28f010); // 4th, load $8, [0x1010 + $1]
    //rep = mm_read(0x1f801010);
    //LONGS_EQUAL(0x13243f, rep);
}

TEST(cpu_instruction, ori){
    cpu_execute(0x3c080013); // $8 = 0x13 << 16
    cpu_execute(0x3508243f); // $8 |= 0x234f
    uint32_t rep = cpu_read_reg(0x8); // $8 == 0x13234f
    LONGS_EQUAL(0x13243f, rep);
}

TEST(cpu_instruction, sll){
    cpu_write_reg(1, 0x10);
    cpu_execute((0x1 << 16) | 0x2 << 11 | 0x3 << 6); // $2 = $1 << 3
    LONGS_EQUAL(0x10 << 3, cpu_read_reg(2));
    cpu_execute(0x00000000); // $0 = $0 << 0
    LONGS_EQUAL(0, cpu_read_reg(0));
}


TEST(cpu_instruction, addiu){
    cpu_write_reg(2, 10);
    cpu_write_reg(3, 5);
    cpu_execute((0x9 << 26) | (0x2 << 21) | (0x3 << 16) | 20); // $2 = $1 << 3
    LONGS_EQUAL(30, cpu_read_reg(3));

    cpu_write_reg(2, 0xFFFFFFFF);
    cpu_execute((0x9 << 26) | (0x2 << 21) | (0x3 << 16) | 0x2); // $2 = $1 << 3
    LONGS_EQUAL( 1, cpu_read_reg(3));
    // FIXME: Must check exception when overflow exception is implemented
}

TEST(cpu_instruction, jump){
    cpu_execute(0x0bf00054); // jump
    CHECK_FALSE(0xbfc00150 == cpu_read_reg(CPU_PC)); // jump is not applied directely
    cpu_execute(0x00000000); // nop
    LONGS_EQUAL(0xbfc00150, cpu_read_reg(CPU_PC));
}

TEST(cpu_instruction, bne){
    cpu_write_reg(10, 0);
    cpu_write_reg(11, 1);
    cpu_execute(0x154bfff7); // BNE $10, $11, -36
    cpu_execute(0x00000000); // nop
    LONGS_EQUAL(0xbfc00000 -36, cpu_read_reg(CPU_PC)); // branching must occurs since 0 != 1

    cpu_write_reg(10, 1);
    cpu_write_reg(CPU_PC, 0xbfc00000); // revert to default
    cpu_execute(0x154bfff7); // BNE $10, $11, -36
    cpu_execute(0x00000000); // nop
    LONGS_EQUAL(0xbfc00008 , cpu_read_reg(CPU_PC)); // branching must not occurs
}

TEST(cpu_instruction, mtc0){
    cpu_write_reg(12, 0x00010000);
    cpu_execute(0x408c6000);
    LONGS_EQUAL(0x00010000, cp0_read_reg(12));
}
