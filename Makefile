
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


lp_sdp8: $(SRC_LIST)
	$(CC) -g $(MOSEK_IPATHS) $(MOSEK_LPATHS) -o lp_sdp8 $(SRC_LIST) -lmosek64 $(LIBS)

lo1: lo1.c
	$(CC) -g $(MOSEK_IPATHS) $(MOSEK_LPATHS) -o lo1 lo1.c -lmosek64 $(LIBS)

easy: 
	  rm -f lp_sdp8
	  make lp_sdp8
	  ./lp_sdp8 ./resource/instance7.txt 3 -3 1 4 1 0 2 10

easyTriangle: 
	  ./lp_sdp8 ./resource/instance5.txt 3 1 1 0 0 0 0 10

easy20: 
	  ./lp_sdp8 ./resource/instance20.txt 3 -3 1 0 0 0 0 10

easy7: 
	  ./lp_sdp8 ./resource/instance7.txt 3 -3 1 4 1 0 2 10

easybqp50: 
	  ./lp_sdp8 ./resource/bqp50_6.txt 3 -3 1 1 5 2 1 3



.PHONY: clean test all

all: lp_sdp8

clean:
	rm -f *.o
