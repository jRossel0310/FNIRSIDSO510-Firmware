CC     ?= cc
INC     = -I../fw/core -I../fw/common -I../fw/storage -I../fw/third_party/fatfs
CFLAGS  = -std=c99 -D_DEFAULT_SOURCE -Wall -Wextra -Werror -O1 -g $(INC) -DFF_USE_MKFS=1
SRCS    = $(wildcard *.c) $(wildcard ../fw/core/*.c) $(wildcard ../fw/storage/*.c)
HDRS    = $(wildcard *.h) $(wildcard ../fw/core/*.h) $(wildcard ../fw/storage/*.h) ../fw/common/spiflash.h

run: build/tests
	./build/tests

test: run

build/ff.o: ../fw/third_party/fatfs/ff.c ../fw/third_party/fatfs/ffconf.h | build
	$(CC) -std=c99 -O1 -g -w $(INC) -DFF_USE_MKFS=1 -c $< -o $@

build/tests: $(SRCS) $(HDRS) build/ff.o | build
	$(CC) $(CFLAGS) $(SRCS) build/ff.o -lm -o $@

build:
	mkdir -p build

clean:
	rm -rf build

.PHONY: run test clean
