#
# Makefile for busy-wait IO tests
#
XCC=gcc
AS=as
LD=ld
CFLAGS=-c -fPIC -Wall -I. -I../include/ -mcpu=arm920t -msoft-float 
# -g: include hooks for gdb
# -c: only compile
# -mcpu=arm920t: generate code for the 920t architecture
# -fpic: emit position-independent code
# -Wall: report all warnings

ASFLAGS	= -mcpu=arm920t -mapcs-32
# -mapcs: always generate a complete stack frame

LDFLAGS = -init main -N -T orex.ld -L/u/wbcowan/gnuarm-4.0.2/lib/gcc/arm-elf/4.0.2 -L../../lib

all:  kern.s main_func.s main_func.elf

bwio.s: bwio.c
	$(XCC) -S $(CFLAGS) bwio.c

kern.s: kern.c 
	$(XCC) -S $(CFLAGS) kern.c

main_func.s: main_func.c 
	$(XCC) -S $(CFLAGS) main_func.c




bwio.o: bwio.s
	$(AS) $(ASFLAGS) -o bwio.o bwio.s

kern.o: kern.s
	$(AS) $(ASFLAGS) -o kern.o kern.s

context_switch.o: context_switch.s
	$(AS) $(ASFLAGS) -o context_switch.o context_switch.s

main_func.o: main_func.s
	$(AS) $(ASFLAGS) -o main_func.o main_func.s




# MUST add .o file here:
main_func.elf: main_func.o context_switch.o bwio.o kern.o 
# MUST add .o file here:
	$(LD) $(LDFLAGS) -o $@ bwio.o kern.o main_func.o context_switch.o -lgcc #-lbwio

clean:
	-rm -f *.elf *.s *.o *.map
