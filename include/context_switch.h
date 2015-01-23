#ifndef __CONTEXT_SWITCH_H__
#define __CONTEXT_SWITCH_H__


void kernel_exit(int retval, unsigned int *sp, unsigned int spsr);

void init_kernelentry();

void init_regs();

#endif
