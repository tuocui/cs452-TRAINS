#include <syscall.h>
#include "tools.h"

int Create( int priority, void (*code) ( ) ) {
  register int retval_reg asm("r0");
  int retval;
  asm volatile(
    "stmfd sp!, {r0, r1}\n\t"
    "swi %1\n\t"
    "mov %0, r0\n\t"
    "ldmfd sp!, {r1, r2}\n\t" 
    : "+r"(retval_reg)
    : "i"(SYS_CREATE)
  );
  retval = retval_reg;
  return retval;
}

int Send( int tid, char *msg, int msglen, char *reply, int replylen ) {
  /* if more(less) local varibles are declared, we need to change
   * the offset of the fp below
   */
  register int retval_reg asm("r0");
  int retval;
  
  asm volatile(
    /* store arguments in ascending order regarding the stack */
    "ldr r0, [fp, #4]\n\t"
    "stmfd sp!, {r0}\n\t"
    "ldr r0, [fp, #-32]\n\t"
    "stmfd sp!, {r0}\n\t"
    "ldr r0, [fp, #-28]\n\t"
    "stmfd sp!, {r0}\n\t"
    "ldr r0, [fp, #-24]\n\t"
    "stmfd sp!, {r0}\n\t"
    "ldr r0, [fp, #-20]\n\t"
    "stmfd sp!, {r0}\n\t"
    "swi %1\n\t"
    "mov %0, r0\n\t"
    "add sp, sp, #20\n\t"
    : "+r"(retval_reg)
    : "i"(SYS_SEND)
  );

  retval = retval_reg;

  return retval;
}


int MyTid( ) {
  register int retval_reg asm("r0");
  int retval;
  asm volatile(
    "swi %1\n\t"
    "mov %0, r0\n\t"
    : "+r"(retval_reg)
    : "i"(SYS_MY_TID)
  );
  retval = retval_reg;
  return retval;
}

int MyParentTid( ) {
  register int retval_reg asm("r0");
  int retval;
  asm volatile(
    "swi %1\n\t"
    "mov %0, r0\n\t"
    : "+r"(retval_reg)
    : "i"(SYS_MY_PARENT_TID)
  );
  retval = retval_reg;
  return retval;
}

void Pass( ) {
  // Lol, super simple
  asm volatile(
   "swi %0\n\t"
   :: "i"(SYS_PASS)
  );
}

void Exit( ) {
  // Also super simple
  asm volatile(
    "swi %0\n\t"
    :: "i"(SYS_KILL)
  );
}

