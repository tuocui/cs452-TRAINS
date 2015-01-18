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
  int i = 0;
  int j = 1;
  int l = 2;

  bwprintf(COM2,"i addr: %x\n\r", &i);
  bwprintf(COM2,"j addr: %x\n\r", &j);
  bwprintf(COM2,"l addr: %x\n\r", &l);
  while (1) {
    bwprintf(COM2, "FIRST USER TASK!! #%d\n\r", ++i);
    asm("swi 0");
  }
}

int main(int argc, char *argv[]) {
  //register unsigned int *lr_reg asm("r0");
  //unsigned int *redboot_lr;
  //asm volatile(
  //  "mov %0, lr\n\t" 
  //: "+r"(lr_reg)
  //);
  //redboot_lr = lr_reg;
  unsigned int td_glb_id;
  unsigned int user_space[100];
  unsigned int checker;


  bwprintf(COM2,"td_glob_id addr: %x\n\r", &td_glb_id);
  bwprintf(COM2,"user_space[-1]: %x\n\r", &user_space[-1]);
  bwprintf(COM2, "user_space[0]: %x\n\r", user_space);
  bwprintf(COM2, "user_space[1]: %x\n\r", &user_space[1]);
  bwprintf(COM2, "checker: %x\n\r", &checker);
  

  int initbuf[999];
  //bwprintf( COM2, "Enter main, addr: %x\r\n", ((int)&main) + 0x218000 );

  //print_pc();
  init_kernelentry();	

  bwprintf(COM2, "Hellow\n\r");

  char ch = 'A';
  int * sp;
  sp = initbuf + 100;
  bwprintf(COM2, "We set char: %c\n\r", ch);
  int * pc = (sp - 10);
  *pc = (int)&first_task + 0x218000;
  int * fp = (sp - 2);
  *fp = (int)sp;
  bwprintf( COM2, "sp: %x, pc: %x, fp: %x, pc_val: %x, fp_val: %x", sp, pc, fp, *pc, *fp );
  bwprintf( COM2, "first_task location: %x\r\n\n\n", &first_task );

  kernel_exit(1, (int)pc, 0xd0);

  register int new_sp_reg asm("r0"); // absolute 
  int new_sp;

  while(1) {
    asm volatile(
      "mov %0, r1\n\t" 
    : "+r"(new_sp_reg)
    );
    new_sp = new_sp_reg;
    bwprintf( COM2, "new_sp:%d\r\n", new_sp ); 
    kernel_exit(1, (int)new_sp, 0xd0);
  }

  bwprintf(COM2, "\n\rGoodbye\n\r");
 
  //lr_reg = redboot_lr;
  //asm volatile(
  //  "mov lr, %0\n\t" 
  //: "+r"(lr_reg)
  //);
  return 0;
}



