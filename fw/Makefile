# Build:  make APP=keytest      (apps/<APP>/main.c + common/*)
# Output: build/APP_DSO-510_<APP>.bin  -> copy onto the scope's IAP drive
APP     ?= keytest
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
CFLAGS  = -mcpu=cortex-m4 -mthumb -O2 -g -Wall -Wextra -ffreestanding -ffunction-sections -fdata-sections -fno-tree-loop-distribute-patterns -Icommon -Icore
LDFLAGS = -nostdlib -T common/link.ld -Wl,--gc-sections -Wl,-Map=build/$(APP).map

SRCS = apps/$(APP)/main.c $(wildcard common/*.c) $(wildcard core/*.c)
# hello is standalone (no common lib) apart from startup/link
ifeq ($(APP),hello)
SRCS = apps/hello/main.c common/startup.c
endif

ELF = build/$(APP).elf
BIN = build/APP_DSO-510_$(APP).bin

all: $(BIN)

$(ELF): $(SRCS) $(wildcard common/*.h) $(wildcard core/*.h) common/link.ld | build
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRCS) -lgcc -o $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@
	arm-none-eabi-size $<

build:
	mkdir -p build

clean:
	rm -rf build

.PHONY: all clean
