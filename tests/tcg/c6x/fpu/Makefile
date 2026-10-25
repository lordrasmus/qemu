# C674x FPU tests: expected values from the special-case rules in TI's
# instruction set reference (SPRUFE8B), see README.
#
# SPDX-License-Identifier: GPL-2.0-or-later

CROSS_COMPILE ?= tic6x-openadk-uclinux-
CC      = $(CROSS_COMPILE)gcc
PYTHON ?= python3
QEMU   ?= qemu-system-c6x

# Inline asm only; -O1 keeps the wrappers from being merged.
CFLAGS ?= -O1

# Bare metal on qemu's dsk6455.  For a board, override UART_BASE and drop
# EXIT_PORT, and set RAM_BASE/RAM_SIZE (see README).
BM_DEFS ?= -DUART_BASE=0x02b80000 -DEXIT_PORT=0x02bc0000
BM_LDFLAGS ?=
BM_CFLAGS = $(CFLAGS) -mno-dsbt -msdata=none -nostdlib -static -fno-builtin

all: fpu-baremetal.elf fpu-linux

fpu-tests.c: gen.py
	$(PYTHON) -I gen.py > $@

fpu-baremetal.elf: main.c fpu-tests.c fpu.h baremetal/crt0.S baremetal/io.c baremetal/link.ld
	$(CC) $(BM_CFLAGS) $(BM_DEFS) -T baremetal/link.ld $(BM_LDFLAGS) \
		baremetal/crt0.S baremetal/io.c main.c fpu-tests.c -lgcc -o $@

fpu-linux: main.c fpu-tests.c fpu.h
	$(CC) $(CFLAGS) main.c fpu-tests.c -o $@

run-qemu: fpu-baremetal.elf
	$(QEMU) -M dsk6455 -nographic -kernel $<

clean:
	rm -f fpu-tests.c fpu-baremetal.elf fpu-linux

.PHONY: all run-qemu clean
