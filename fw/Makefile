# Build:  make APP=<name>     name = app (the firmware) or a test app in apps/<name>
# Output: build/APP_DSO-510_<APP>.bin  -> copy onto the scope's IAP drive (tools/flash.sh)
APP     ?= app
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
ARCH    = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS  = $(ARCH) -O2 -g -Wall -Wextra -ffreestanding -ffunction-sections -fdata-sections \
          -fno-tree-loop-distribute-patterns -Icommon -Icore -Iapp -Istorage -Ithird_party/fatfs
CFLAGS += $(EXTRA)
LDFLAGS = -nostdlib -T common/link.ld -Wl,--gc-sections -Wl,-Map=build/$(APP).map

COMMON  = $(wildcard common/*.c)
CORE    = $(wildcard core/*.c)
STORAGE = $(wildcard storage/*.c)
FF_OBJ  = build/ff.o
ifeq ($(APP),hello)
SRCS = apps/hello/main.c common/startup.c
OBJS =
else ifeq ($(APP),app)
SRCS = $(wildcard app/*.c) $(COMMON) $(CORE) $(STORAGE)
OBJS = $(FF_OBJ)
else
SRCS = apps/$(APP)/main.c $(COMMON) $(CORE) $(STORAGE)
OBJS = $(FF_OBJ)
endif

ELF = build/$(APP).elf
BIN = build/APP_DSO-510_$(APP).bin
APPS = $(notdir $(wildcard apps/*)) $(if $(wildcard app/main.c),app)
HDRS = $(wildcard common/*.h core/*.h app/*.h storage/*.h third_party/fatfs/*.h)

all: $(BIN)

$(FF_OBJ): third_party/fatfs/ff.c third_party/fatfs/ffconf.h third_party/fatfs/ff.h | build
	$(CC) $(CFLAGS) -w -c $< -o $@

$(ELF): $(SRCS) $(OBJS) $(HDRS) common/link.ld | build
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRCS) $(OBJS) -lgcc -o $@

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
