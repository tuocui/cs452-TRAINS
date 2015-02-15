#
# Makefile
# extra flagx are set by nmake
#

# include new dir here:
SRCS_KERN=$(wildcard src/kern/*.c) 
SRCS_USER=$(wildcard src/user/*.c) 
SRCS_SERV=$(wildcard src/serv/*.c) 
SRCS_UTIL=$(wildcard src/util/*.c) 

ASMS_KERN=$(addprefix build/kern/,$(notdir $(SRCS_KERN:.c=.s))) build/kern/context_switch.s
ASMS_USER=$(addprefix build/user/,$(notdir $(SRCS_USER:.c=.s))) 
ASMS_SERV=$(addprefix build/serv/,$(notdir $(SRCS_SERV:.c=.s))) 
ASMS_UTIL=$(addprefix build/util/,$(notdir $(SRCS_UTIL:.c=.s))) 

OBJS_KERN=$(addprefix build/kern/,$(notdir $(SRCS_KERN:.c=.o))) build/kern/context_switch.o
OBJS_USER=$(addprefix build/user/,$(notdir $(SRCS_USER:.c=.o))) 
OBJS_SERV=$(addprefix build/serv/,$(notdir $(SRCS_SERV:.c=.o))) 
OBJS_UTIL=$(addprefix build/util/,$(notdir $(SRCS_UTIL:.c=.o))) 

ASMS=$(ASMS_KERN) $(ASMS_USER) $(ASMS_SERV) $(ASMS_UTIL)
OBJS=$(OBJS_KERN) $(OBJS_USER) $(OBJS_SERV) $(OBJS_UTIL)

ELF=elf/kernel.elf


XCC=gcc
AS=as
LD=ld
CFLAGS=-c ${DEBUG} ${CACHE} ${AST} -fPIC -Wall -Werror -I./include -I./include/kern/ -I./include/user/ -I./include/serv/ -I./include/util/ -mcpu=arm920t -msoft-float 
CFLAGS_O=-c ${DEBUG} ${CACHE} ${AST} ${OPT} ${OPT_LEVEL} -fPIC -Wall -Werror -I./include -I./include/kern/ -I./include/user/ -I./include/serv/ -I./include/util/ -mcpu=arm920t -msoft-float 
ASFLAGS	= -mcpu=arm920t -mapcs-32
LDFLAGS = -init main -N -T orex.ld -L/u/wbcowan/gnuarm-4.0.2/lib/gcc/arm-elf/4.0.2 -L../../lib
# -g: include hooks for gdb
# -c: only compile
# -mcpu=arm920t: generate code for the 920t architecture
# -fpic: emit position-independent code
# -Wall: report all warnings
# -mapcs: always generate a complete stack frame


.PHONY: clean all install

all: $(ASMS) $(ELF)

clean:
	-rm -f build/kern/* build/user/* build/serv/* build/util/* elf/*

build/%.c: src/%.c
	cp -f $< $@
	cp -f src/kern/context_switch.s.in build/kern/context_switch.s

build/%.s: build/%.c
	$(XCC) -S $(CFLAGS_O) -o $@ $< 


build/%.o: build/%.s
	$(AS) $(ASFLAGS) -o $@ $<
build/kern/context_switch.o: build/kern/context_switch.s
	$(AS) $(ASFLAGS) -o $@ $<


$(ELF): $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) -lgcc #-lbwio
