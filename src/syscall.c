#include <syscall.h>

int Create( int priority, void (*code) ( ) ) {
  register int retval_reg asm("r0");
  int retval;
  asm volatile(
    "stmfd sp!, {r0, r1}\n\t"
    "swi 1\n\t"
    "mov %0, r0\n\t"
    "ldmfd sp!, {r1, r2}\n\t"
    : "+r"(retval_reg)
  );
  retval = retval_reg;
  return retval;
}

int MyTid( ) {
  register int retval_reg asm("r0");
  int retval;
  asm volatile(
    "swi 2\n\t"
    "mov %0, r0\n\t"
    : "+r"(retval_reg)
  );
  retval = retval_reg;
  return retval;
}

int MyParentTid( ) {
  register int retval_reg asm("r0");
  int retval;
  asm volatile(
    "swi 3\n\t"
    "mov %0, r0\n\t"
    : "+r"(retval_reg)
  );
  retval = retval_reg;
  return retval;
}

void Pass( ) {
  // Lol, super simple
  asm volatile(
   "swi 4\n\t"
  );
}

void Exit( ) {
  // Also super simple
  asm volatile(
    "swi 99\n\t"
  );
}
