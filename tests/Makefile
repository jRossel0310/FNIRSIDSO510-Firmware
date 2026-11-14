CC     ?= cc
CFLAGS  = -std=c99 -D_DEFAULT_SOURCE -Wall -Wextra -Werror -O1 -g -I../fw/core -I../fw/common
SRCS    = $(wildcard *.c) $(wildcard ../fw/core/*.c)
HDRS    = $(wildcard *.h) $(wildcard ../fw/core/*.h)

run: build/tests
	./build/tests

build/tests: $(SRCS) $(HDRS)
	mkdir -p build
	$(CC) $(CFLAGS) $(SRCS) -lm -o $@

clean:
	rm -rf build

test: run

.PHONY: run test clean
