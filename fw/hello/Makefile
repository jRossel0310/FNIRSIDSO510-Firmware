CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
CFLAGS  = -mcpu=cortex-m4 -mthumb -O2 -g -Wall -Wextra -ffreestanding -ffunction-sections -fdata-sections
LDFLAGS = -nostdlib -T link.ld -Wl,--gc-sections -Wl,-Map=hello.map

OUT = APP_DSO-510_HELLO.bin

all: $(OUT)

hello.elf: main.c startup.c link.ld
	$(CC) $(CFLAGS) $(LDFLAGS) main.c startup.c -o $@

$(OUT): hello.elf
	$(OBJCOPY) -O binary $< $@
	arm-none-eabi-size $<

clean:
	rm -f hello.elf hello.map $(OUT)

.PHONY: all clean
