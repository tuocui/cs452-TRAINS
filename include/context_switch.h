#ifndef __CONTEXT_SWITCH_H__
#define __CONTEXT_SWITCH_H__


#include "bwio.h"

void print_pc();

void kernel_exit(int retval, int sp, int spsr);

void init_kernelentry();

#endif
