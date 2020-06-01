
#
# Compile and run MOSEK examples
#

CC=g++ -m64
MOSEK_IPATHS=-I ../../mosek/9.1/tools/platform/linux64x86/h
MOSEK_BINPATH=../../mosek/9.1/tools/platform/linux64x86/bin
MOSEK_LPATHS=-L$(MOSEK_BINPATH) -Wl,-rpath-link,$(MOSEK_BINPATH) '-Wl,-rpath=$$ORIGIN/$(MOSEK_BINPATH)'
LIBS=-lm

SRC_DIR     = ./src
BUILD_DIR   = ./build
BIN_DIR     = ./bin
#rwildcard   = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
#rwildcard=$(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(filter $(subst *,%,$2),$d))
#SRC_LIST    = $(call rwildcard, ./src/, *.cpp)
SRC_LIST = $(wildcard $(SRC_DIR)/*cpp) $(wildcard $(SRC_DIR)/*/*.cpp)
OBJ_LIST    = $(BUILD_DIR)/$(notdir $(SRC_LIST:.cpp=.o))


bb_mkc_run: $(SRC_LIST)
	$(CC) -g $(MOSEK_IPATHS) $(MOSEK_LPATHS) -o bb_mkc_run $(SRC_LIST) -lmosek64 $(LIBS)

install: 
	  rm -f bb_mkc_run
	  make bb_mkc_run

run_instance7:
	  ./bb_mkc_run ./resource/instance7.txt 3

run_instance7_with_partitions:
	  ./bb_mkc_run ./resource/instance7.txt 3 

run_instance5_partitions_in_ProblemParameters:
	  ./bb_mkc_run ./resource/instance5.txt

run_instance20_with_partition:
	  ./bb_mkc_run ./resource/instance20.txt 5

run_instance20_partition_in_parameters:
	  ./bb_mkc_run ./resource/instance20.txt

run: 
	  ./bb_mkc_run 

.PHONY: clean test all

all: bb_mkc_run run

clean:
	rm -f *.o
