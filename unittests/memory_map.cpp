#include <CppUTest/TestHarness.h>

extern "C"
{
#include "memory_map.h"
#include "unistd.h"
}

TEST_GROUP(memory_map){};


TEST(memory_map, load_bios){
    uint32_t rep = mm_load_bios("bios/scph5502.bin");
    LONGS_EQUAL(MM_FAILURE, rep);

    rep = mm_initialise();
    LONGS_EQUAL(MM_SUCCESS, rep);

    rep = mm_load_bios("somewhere/unknown");
    LONGS_EQUAL(MM_FAILURE, rep);

    rep = mm_initialise();
    LONGS_EQUAL(MM_FAILURE, rep);

    rep = mm_load_bios("bios/scph5502.bin");
    LONGS_EQUAL(MM_SUCCESS, rep);
}
