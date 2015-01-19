#include <tools.h>
#include <kernel.h>
#include <task_descriptor.h>

void first_task() {
  int i = 0;
  while (1) {
    bwprintf(COM2, "FIRST USER TASK!! #%d\n\r", ++i);
    asm("swi 0");
  }
}
void second_task() {
  int i = 0;
  while (1) {
    bwprintf(COM2, "SECOND USER TASK!! #%d\n\r", ++i);
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
  
  global_context_t gc;
  tds_init(&gc);
  init_kernelentry();

  //task_descriptor_t *first_td = tds_create_td(&gc, 1, (int*)( (int) &first_task + REDBOOT_OFFSET));
 // task_descriptor_t *second_td = tds_create_td(&gc, 1, (int*)( (int) &second_task + REDBOOT_OFFSET));

  bwprintf(COM2, "Hellow\n\r");

  assert(1==2, "this is a msg");

 register unsigned int *new_sp_reg asm("r0"); // absolute 
 unsigned int *new_sp;

 int i = 0;
 while (1){
   task_descriptor_t *first_td = tds_create_td(&gc, 1, (int*)( (int) &first_task + REDBOOT_OFFSET));
   if (first_td == NULL) {
     break;    
   }
   kernel_exit(1, first_td->sp, 0xd0);
   asm volatile(
     "mov %0, r1\n\t" 
   : "+r"(new_sp_reg)
   );
   new_sp = new_sp_reg;
   bwprintf( COM2, "new_sp1:%d\r\n", new_sp ); 
   first_td->sp = new_sp;

   bwprintf( COM2, "i:%d\r\n", i ); 

   int test;
   bwprintf( COM2, "test_addr:%d\r\n", &test ); 
   i++;  
 }

 bwprintf(COM2, "\n\rGoodbye\n\r");

 //lr_reg = redboot_lr;
 //asm volatile(
 //  "mov lr, %0\n\t" 
 //: "+r"(lr_reg)
 //);
  return 0;
}



