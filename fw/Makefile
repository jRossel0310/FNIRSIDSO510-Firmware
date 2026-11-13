# Build:  make APP=<name>     name = app (the firmware) or a test app in apps/<name>
# Output: build/APP_DSO-510_<APP>.bin  -> copy onto the scope's IAP drive (tools/flash.sh)
APP     ?= app
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
ARCH    = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS  = $(ARCH) -O2 -g -Wall -Wextra -ffreestanding -ffunction-sections -fdata-sections \
          -fno-tree-loop-distribute-patterns -Icommon -Icore -Iapp
CFLAGS += $(EXTRA)
LDFLAGS = -nostdlib -T common/link.ld -Wl,--gc-sections -Wl,-Map=build/$(APP).map

COMMON = $(wildcard common/*.c)
CORE   = $(wildcard core/*.c)
ifeq ($(APP),hello)
SRCS = apps/hello/main.c common/startup.c
else ifeq ($(APP),app)
SRCS = $(wildcard app/*.c) $(COMMON) $(CORE)
else
SRCS = apps/$(APP)/main.c $(COMMON) $(CORE)
endif

ELF = build/$(APP).elf
BIN = build/APP_DSO-510_$(APP).bin
APPS = $(notdir $(wildcard apps/*)) $(if $(wildcard app/main.c),app)

all: $(BIN)

$(ELF): $(SRCS) $(wildcard common/*.h core/*.h app/*.h) common/link.ld | build
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRCS) -lgcc -o $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@
	arm-none-eabi-size $<

build:
	mkdir -p build

everything:
	for a in $(APPS); do $(MAKE) --no-print-directory APP=$$a || exit 1; done

clean:
	rm -rf build

.PHONY: all clean everything
