# Screen emulator: builds the firmware's UI code for the PC and renders the README screenshots.
#   make -C sim            -> docs/screens/*.png (needs python3 with Pillow for the PNG step)
CC     ?= cc
INC     = -I../fw/core -I../fw/common -I../fw/app
CFLAGS  = -std=c99 -D_DEFAULT_SOURCE -Wall -Wextra -Werror -O1 $(INC)
CORE    = $(filter-out ../fw/core/msc.c ../fw/core/usb_std.c,$(wildcard ../fw/core/*.c))
SRCS    = screens.c lcd_sim.c ../fw/app/ui.c $(CORE)
OUT     = ../docs/screens

all: png

build/screens: $(SRCS) $(wildcard ../fw/core/*.h ../fw/app/*.h ../fw/common/*.h) | build
	$(CC) $(CFLAGS) $(SRCS) -lm -o $@

build:
	mkdir -p build

ppm: build/screens
	mkdir -p build/ppm
	./build/screens build/ppm

png: ppm
	mkdir -p $(OUT)
	python3 -c "import sys, glob, os; from PIL import Image; [Image.open(f).save(os.path.join('$(OUT)', os.path.basename(f)[:-4] + '.png'), optimize=True) or print('->', os.path.basename(f)[:-4] + '.png') for f in sorted(glob.glob('build/ppm/*.ppm'))]"

clean:
	rm -rf build

.PHONY: all ppm png clean
