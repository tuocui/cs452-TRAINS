#
# Makefile for kernel 
# DEBUG flag is set by nmake
#

XCC=gcc
AS=as
LD=ld
CFLAGS=-c ${DEBUG} -fPIC -Wall -Werror -I. -I../include/ -mcpu=arm920t -msoft-float 
# -g: include hooks for gdb
# -c: only compile
# -mcpu=arm920t: generate code for the 920t architecture
# -fpic: emit position-independent code
# -Wall: report all warnings

ASFLAGS	= -mcpu=arm920t -mapcs-32
# -mapcs: always generate a complete stack frame

LDFLAGS = -init main -N -T orex.ld -L/u/wbcowan/gnuarm-4.0.2/lib/gcc/arm-elf/4.0.2 -L../../lib


all:  kernel.s kernel.s kernel.elf


bwio.s: bwio.c
	$(XCC) -S $(CFLAGS) bwio.c

kernel.s: kernel.c 
	$(XCC) -S $(CFLAGS) kernel.c

task_descriptor.s: task_descriptor.c 
	$(XCC) -S $(CFLAGS) task_descriptor.c

scheduler.s: scheduler.c 
	$(XCC) -S $(CFLAGS) scheduler.c

syscall.s: syscall.c 
	$(XCC) -S $(CFLAGS) syscall.c

kernel_syscall.s: kernel_syscall.c 
	$(XCC) -S $(CFLAGS) kernel_syscall.c

user_task.s:user_task.c 
	$(XCC) -S $(CFLAGS) user_task.c


bwio.o: bwio.s
	$(AS) $(ASFLAGS) -o bwio.o bwio.s

context_switch.o: context_switch.s
	$(AS) $(ASFLAGS) -o context_switch.o context_switch.s

kernel.o: kernel.s
	$(AS) $(ASFLAGS) -o kernel.o kernel.s

task_descriptor.o: task_descriptor.s
	$(AS) $(ASFLAGS) -o task_descriptor.o task_descriptor.s

scheduler.o: scheduler.s
	$(AS) $(ASFLAGS) -o scheduler.o scheduler.s

syscall.o: syscall.s
	$(AS) $(ASFLAGS) -o syscall.o syscall.s

kernel_syscall.o: kernel_syscall.s
	$(AS) $(ASFLAGS) -o kernel_syscall.o kernel_syscall.s

user_task.o: user_task.s
	$(AS) $(ASFLAGS) -o user_task.o user_task.s


# MUST add .o file here:
kernel.elf: kernel.o context_switch.o task_descriptor.o scheduler.o syscall.o kernel_syscall.o user_task.o bwio.o
# MUST add .o file here:
	$(LD) $(LDFLAGS) -o $@ bwio.o task_descriptor.o scheduler.o syscall.o kernel_syscall.o user_task.o context_switch.o kernel.o  -lgcc #-lbwio

clean:
	-rm -f *.elf *.s *.o *.map ../obj/* ../asm/* ../elf/*
