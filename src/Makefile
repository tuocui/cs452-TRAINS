#
#
# Makefile for kernel 
# DEBUG flag is set by nmake
#

# MUST add .c file here:
SRCS=clock_server.c bwio.c kernel.c task_descriptor.c scheduler.c syscall.c kernel_syscall.c user_task.c nameserver.c rps.c 
_ASMS=$(SRCS:.c=.s) 
ASMS=$(_ASMS) context_switch.s
OBJS=$(SRCS:.c=.o) context_switch.o

XCC=gcc
AS=as
LD=ld
CFLAGS=-c ${DEBUG} ${CACHE} ${AST} -fPIC -Wall -Werror -I. -I../include/ -mcpu=arm920t -msoft-float 
CFLAGS_O=-c ${DEBUG} ${CACHE} ${AST} ${OPT} ${OPT_LEVEL} -fPIC -Wall -Werror -I. -I../include/ -mcpu=arm920t -msoft-float 
# -g: include hooks for gdb
# -c: only compile
# -mcpu=arm920t: generate code for the 920t architecture
# -fpic: emit position-independent code
# -Wall: report all warnings

ASFLAGS	= -mcpu=arm920t -mapcs-32
# -mapcs: always generate a complete stack frame

LDFLAGS = -init main -N -T orex.ld -L/u/wbcowan/gnuarm-4.0.2/lib/gcc/arm-elf/4.0.2 -L../../lib

.PHONY: clean, all


all: $(_ASMS) kernel.elf

%.s: %.c
	$(XCC) -S $(CFLAGS_O) $<

%.o: %.s
	$(AS) $(ASFLAGS) -o $@ $<

kernel.elf: $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) -lgcc #-lbwio

clean:
	-rm -f *.elf *.s *.o *.map ../obj/* ../asm/* ../elf/*
