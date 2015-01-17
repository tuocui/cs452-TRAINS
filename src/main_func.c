#include "kernel.h"
#include "context_switch.h"

void second_task() {
  bwprintf(COM2, "SECONDUSER TASK!!\n\r");
  asm("swi 0");
}


void first_task() {
  bwprintf(COM2, "FIRST USER TASK!!\n\r");
  //asm("swi 0");
}

int main(int argc, char *argv[]) {
  int initbuf[9999];
  bwprintf( COM2, "Enter main" );

  print_pc();
  init_kernelentry();

  bwprintf(COM2, "Hellow\n\r");

  char ch = 'A';
  int * sp;
  sp = initbuf + 1000;
  bwprintf(COM2, "We set char: %c\n\r", ch);
  int * lr = (sp + 14);
  *lr = (int)&first_task;

  /*lr = (sp + 15);
  *lr = (int)&first_task;
  lr = (sp + 13);
  *lr = (int)&first_task;
  lr = (sp - 13);
  *lr = (int)&first_task;
  lr = (sp - 14);
  *lr = (int)&first_task;
  lr = (sp - 15);
  *lr = (int)&first_task;*/

  kernel_exit(1, (int)lr, 0xd0);
  bwprintf(COM2, "\n\rGoodbye\n\r");
  return 0;
}



