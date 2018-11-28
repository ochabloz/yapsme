# Copyright 2018 Olivier CHABLOZ
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# C compilation definition
CC      := cc
CFLAGS    := -g -Wall
CFLAGS  += -std=gnu99

# C++ compilation definition
CPP     := g++
CPPFLAGS  := -g -Wall
CPPFLAGS  += -I $(CPPUTEST_HOME)/include

LDFLAGS :=

SRC_DIR := src/
INC_DIR := include/

# obj dir is where compiled files will be generated
OBJ_DIR := objs/
SRC_FILES := $(wildcard $(SRC_DIR)*.c)
OBJ_FILES := $(patsubst $(SRC_DIR)%.c,$(OBJ_DIR)%.o,$(SRC_FILES))



UNITTEST_DIR := unittests/
# list all objs files to include except the main program
OBJ_FILES_FLT := $(patsubst $(SRC_DIR)%.c,$(OBJ_DIR)%.o, $(filter-out $(SRC_DIR)main.c, $(SRC_FILES)))
TST_FILES := $(wildcard $(UNITTEST_DIR)*.cpp)
TST_OBJ_FILES := $(patsubst $(UNITTEST_DIR)%.cpp, $(OBJ_DIR)unittest_%.o,$(TST_FILES))


# objects files compilation
$(OBJ_DIR)%.o: $(SRC_DIR)%.c
	$(CC) $(CFLAGS) -I $(INC_DIR) -c -o $@ $<

$(OBJ_DIR)unittest_%.o: $(UNITTEST_DIR)%.cpp
	$(CPP) $(CPPFLAGS) -I $(INC_DIR) -c -o $@ $<



.PHONY: clean makedir
all: makedir unittest yapsme

test: makedir unittest



unittest: $(OBJ_FILES_FLT) $(TST_OBJ_FILES)
	$(CPP) -o $(OBJ_DIR)$@ $^ $(LDFLAGS) -L$(CPPUTEST_HOME)/lib -lCppUTest
	$(OBJ_DIR)$@ -c



yapsme: $(OBJ_FILES)
	$(CC) -o $(OBJ_DIR)$@ $^ $(LDFLAGS)
	@echo "Now running" $@
	$(OBJ_DIR)$@



makedir:
	mkdir -p $(OBJ_DIR)

clean:
	@echo "clean..."
	rm -rf $(OBJ_DIR)
