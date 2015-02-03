#include <syscall.h>
#include <tools.h>


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
  
#ifndef OPT
  //debug("optimiazaion is on");
  /* without optimization */
  asm volatile(
    /* store arguments in ascending order regarding the stack */
    "ldr r0, [fp, #-20]\n\t"    // tid
    "stmfd sp!, {r0}\n\t"
    "ldr r0, [fp, #4]\n\t"      // replylen
    "stmfd sp!, {r0}\n\t"
    "ldr r0, [fp, #-32]\n\t"    // reply
    "stmfd sp!, {r0}\n\t"
    "ldr r0, [fp, #-28]\n\t"    // msglen
    "stmfd sp!, {r0}\n\t"
    "ldr r0, [fp, #-24]\n\t"    // msg
    "stmfd sp!, {r0}\n\t"
    "swi %1\n\t"
    "mov %0, r0\n\t"
    "add sp, sp, #20\n\t"
    : "+r"(retval_reg)
    : "i"(SYS_SEND)
  );

#else
  /* with optimization */
  asm volatile(
    /* store arguments in ascending order regarding the stack */
    "stmfd sp!, {r0}\n\t"            // tid 
    "ldr r0, [sp, #4]\n\t"           
    "stmfd sp!, {r0}\n\t"            //replylen
    "stmfd sp!, {r1, r2, r3}\n\t"   // reply, msglen, msg
    "swi %1\n\t"
    "mov %0, r0\n\t"
    "add sp, sp, #20\n\t"
    : "+r"(retval_reg)
    : "i"(SYS_SEND)
  );
#endif

  retval = retval_reg;

  return retval;
}

int Receive( int *tid, char *msg, int msglen ) {
  register int retval_reg asm("r0");
  int retval;
  asm volatile( 
      "stmfd sp!, {r0, r1, r2}\n\t"
      "swi %1\n\t"
      "mov %0, r0\n\t"
      "add sp, sp, #12\n\t"
      : "+r"(retval_reg)
      : "i"(SYS_RECEIVE)
      );
  retval = retval_reg;
  return retval;
}

int Reply(int tid, char *reply, int replylen ) {
  register int retval_reg asm("r0");
  int retval;
  asm volatile(
      "stmfd sp!, {r0, r1, r2}\n\t"
      "swi %1\n\t"
      "mov %0, r0\n\t"
      "add sp, sp, #12\n\t"
      : "+r"(retval_reg)
      : "i"(SYS_REPLY)
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
    :: "i"(SYS_EXIT)
  );
}

