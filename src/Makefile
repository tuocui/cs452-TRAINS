#
# Makefile for kernel 
# DEBUG flag is set by nmake
#
SRCS=bwio.c kernel.c task_descriptor.c scheduler.c syscall.c kernel_syscall.c user_task.c nameserver.c rps.c timer.c
ASMS=$(SRCS:.c=.s) context_switch.s
OBJS=$(SRCS:.c=.o) context_switch.o

XCC=gcc
AS=as
LD=ld
CFLAGS=-c ${DEBUG} ${CACHE} -fPIC -Wall -Werror -I. -I../include/ -mcpu=arm920t -msoft-float 
CFLAGS_O=-c ${DEBUG} ${CACHE} ${OPT} ${OPT_LEVEL} -fPIC -Wall -Werror -I. -I../include/ -mcpu=arm920t -msoft-float 
# -g: include hooks for gdb
# -c: only compile
# -mcpu=arm920t: generate code for the 920t architecture
# -fpic: emit position-independent code
# -Wall: report all warnings

ASFLAGS	= -mcpu=arm920t -mapcs-32
# -mapcs: always generate a complete stack frame

LDFLAGS = -init main -N -T orex.ld -L/u/wbcowan/gnuarm-4.0.2/lib/gcc/arm-elf/4.0.2 -L../../lib

.PHONY: clean, all

all:  kernel.s kernel.s kernel.elf

%.s: %.c
	$(XCC) -S $(CFLAGS_O) $<

##%.o: %.s
##	$(AS) $(ASFLAGS) -o $@ $<

bwio.o: bwio.s
	$(AS) $(ASFLAGS) -o $@ $<

context_switch.o: context_switch.s
	$(AS) $(ASFLAGS) -o $@ $<

kernel.o: kernel.s
	$(AS) $(ASFLAGS) -o $@ $<

task_descriptor.o: task_descriptor.s
	$(AS) $(ASFLAGS) -o $@ $<

scheduler.o: scheduler.s
	$(AS) $(ASFLAGS) -o $@ $<

syscall.o: syscall.s
	$(AS) $(ASFLAGS) -o $@ $<

kernel_syscall.o: kernel_syscall.s
	$(AS) $(ASFLAGS) -o $@ $<

user_task.o: user_task.s
	$(AS) $(ASFLAGS) -o $@ $<

nameserver.o: nameserver.s
	$(AS) $(ASFLAGS) -o $@ $<

rps.o: rps.s
	$(AS) $(ASFLAGS) -o $@ $<

timer.o: timer.s
	$(AS) $(ASFLAGS) -o $@ $<


# MUST add .o file here:
#kernel.elf: kernel.o context_switch.o task_descriptor.o scheduler.o syscall.o kernel_syscall.o user_task.o nameserver.o rps.o timer.o bwio.o
kernel.elf: $(OBJS)
# MUST add .o file here:
	$(LD) $(LDFLAGS) -o $@ $(OBJS) -lgcc #-lbwio

clean:
	-rm -f *.elf *.s *.o *.map ../obj/* ../asm/* ../elf/*
