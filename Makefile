#
#
# Makefile for kernel 
# DEBUG flag is set by nmake
#

# MUST add .c file here:
SRCS_KERN=$(wildcard src/*.c) #clock_server.c bwio.c kernel.c task_descriptor.c scheduler.c syscall.c kernel_syscall.c user_task.c nameserver.c rps.c 
ASMS=$(addprefix build/,$(notdir $(SRCS_KERN:.c=.s))) build/context_switch.s
OBJS=$(addprefix build/,$(notdir $(SRCS_KERN:.c=.o))) build/context_switch.o
ELF=elf/kernel.elf

XCC=gcc
AS=as
LD=ld
CFLAGS=-c ${DEBUG} ${CACHE} ${AST} -fPIC -Wall -Werror -I. -I./include/ -mcpu=arm920t -msoft-float 
CFLAGS_O=-c ${DEBUG} ${CACHE} ${AST} ${OPT} ${OPT_LEVEL} -fPIC -Wall -Werror -I. -I./include/ -mcpu=arm920t -msoft-float 
# -g: include hooks for gdb
# -c: only compile
# -mcpu=arm920t: generate code for the 920t architecture
# -fpic: emit position-independent code
# -Wall: report all warnings

ASFLAGS	= -mcpu=arm920t -mapcs-32
# -mapcs: always generate a complete stack frame

LDFLAGS = -init main -N -T orex.ld -L/u/wbcowan/gnuarm-4.0.2/lib/gcc/arm-elf/4.0.2 -L../../lib

.PHONY: clean all install

all: $(ASMS) $(ELF)

build/%.c: src/%.c
	cp -f $< $@
	cp -f src/context_switch.s.in build/context_switch.s

build/%.s: build/%.c
	$(XCC) -S $(CFLAGS_O) -o $@ $< 


#build/%.s.in: src/%.s
#	cp -f $< $@

build/%.o: build/%.s
	$(AS) $(ASFLAGS) -o $@ $<
build/context_switch.o: build/context_switch.s
	$(AS) $(ASFLAGS) -o $@ $<

#src/%.s: src/%.c
#	$(XCC) -S $(CFLAGS_O) $<
#

$(ELF): $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) -lgcc #-lbwio


clean:
	-rm -f build/*.* build/*.elf *.s *.o *.map ../obj/* ../asm/* ../elf/*


