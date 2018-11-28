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
#include <CppUTest/TestHarness.h>

extern "C"
{
#include "memory_map.h"
#include "unistd.h"
}

TEST_GROUP(memory_map){};


TEST(memory_map, load_bios){
    mm_desinit();
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

    mm_desinit();
}


TEST(memory_map, read_bios){
    mm_initialise();
    mm_load_bios("bios/scph5502.bin");
    uint32_t rep = mm_read(0xbfc00000);
    LONGS_EQUAL(0x3c080013, rep); // This is the first value of a bios file
    rep = mm_read(0xbfc00001);
    LONGS_EQUAL(0x3c080013, rep);

    rep = mm_read(0xbfc00004);
    LONGS_EQUAL(0x3508243f, rep);

    mm_desinit();
}


TEST(memory_map, read_write_ram){
    mm_initialise();
    mm_load_bios("bios/scph5502.bin");
    mm_write(0xA0000000, 0xAA);
    LONGS_EQUAL(0xAA, mm_read(0xA0000000));

    LONGS_EQUAL(0x00, mm_read(0xA0000400));
    mm_write(0xA0000400, 0xAA);
    LONGS_EQUAL(0xAA, mm_read(0xA0000400));

    // testing boundaries:
    //mm_write(0xA0200000, 0xAA);
    //LONGS_EQUAL(0xFF, mm_read(0xA0200000));
    mm_write(0xA01fffff, 0xAA);
    LONGS_EQUAL(0xAA, mm_read(0xA01fffff));

    mm_desinit();
}
