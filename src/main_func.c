#include "kernel.h"
#include "context_switch.h"

	void second_task() {
  bwprintf(COM2, "SECONDUSER TASK!!\n\r");
  asm("swi 0");
}

void call_swi() {
  asm("swi 0");
}

void first_task() {
  //int i = 123;
  bwprintf(COM2, "FIRST USER TASK!!\n\r");
  asm("swi 0");
  bwprintf(COM2, "FIRST USER TASK AGAIN!");
  asm("swi 0");
}

int main(int argc, char *argv[]) {
  register unsigned int *lr_reg asm("r0");
  unsigned int *redboot_lr;
  asm volatile(
    "mov %0, lr\n\t" 
  : "+r"(lr_reg)
  );
  redboot_lr = lr_reg;

  int initbuf[999];
  bwprintf( COM2, "Enter main, addr: %x\r\n", ((int)&main) + 0x218000 );

  print_pc();
  init_kernelentry();	

  bwprintf(COM2, "Hellow\n\r");

  char ch = 'A';
  int * sp;
  sp = initbuf + 100;
  bwprintf(COM2, "We set char: %c\n\r", ch);
  int * pc = (sp - 10);
  *pc = (int)&first_task + 0x218000;
  int * fp = (sp - 2);
  *fp = sp;
  bwprintf( COM2, "sp: %x, pc: %x, fp: %x, pc_val: %x, fp_val: %x", sp, pc, fp, *pc, *fp );
  bwprintf( COM2, "first_task location: %x\r\n\n\n", &first_task );

  kernel_exit(1, (int)pc, 0xd0);

  register unsigned int *new_sp_reg asm("r0");
  unsigned int *new_sp;
  asm volatile(
    "mov %0, r1\n\t" 
  : "+r"(new_sp_reg)
  );
  new_sp = new_sp_reg;

  bwprintf(COM2, "\n\rAfter first kernel exit\n\r");
  bwprintf(COM2, "\n\rUser sp: %x\n\r", new_sp);
  bwprintf(COM2, "\n\rUser pc: %x\n\r", *new_sp);

  kernel_exit(1, new_sp, 0xd0);



  bwprintf(COM2, "\n\rGoodbye\n\r");
  bwprintf(COM2, "\n\rGoodbye\n\r");
  bwprintf(COM2, "\n\rGoodbye\n\r");
  bwprintf(COM2, "\n\rGoodbye\n\r");
  bwprintf(COM2, "\n\rGoodbye\n\r");
  bwprintf(COM2, "\n\rGoodbye\n\r");
  bwprintf(COM2, "\n\rGoodbye\n\r");
 
  lr_reg = redboot_lr;
  asm volatile(
    "mov lr, %0\n\t" 
  : "+r"(lr_reg)
  );
  return 0;
}



