#include <tools.h>
#include <kernel.h>
//#include <task_descriptor.h>

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

void kernel_init( global_context_t *gc) {
  gc->cur_task = NULL;
  gc->td_glb_id = 1;
  gc->priority_bitmap = 0;

  tds_init(gc);
  init_kernelentry();
  init_schedulers(gc);
}

int main(int argc, char *argv[]) {
  bwputstr( COM2, "INITIALIZING. Trust us, it's not hanging.\r\n" );
  global_context_t gc;
  kernel_init( &gc );

  register unsigned int *new_sp_reg asm("r1"); // absolute 
  unsigned int *new_sp;
  register unsigned int new_spsr_reg asm("r2"); // absolute 
  unsigned int new_spsr;

  task_descriptor_t *first_td = tds_create_td(&gc, 1, (int*)( (int) &first_task + REDBOOT_OFFSET));
  task_descriptor_t *second_td = tds_create_td(&gc, 2, (int*)( (int) &first_task + REDBOOT_OFFSET));
  task_descriptor_t *third_td = tds_create_td(&gc, 1, (int*)( (int) &second_task + REDBOOT_OFFSET));
  add_to_priority( &gc, second_td->priority, second_td );
  add_to_priority( &gc, third_td->priority, third_td );
  add_to_priority( &gc, first_td->priority, first_td );

  while (1){
    task_descriptor_t *scheduled_td = schedule( &gc );
    // EYYYYYYY, we exit
    if ( scheduled_td == NULL ) {
      break;    
    }
    bwprintf( COM2, "Scheduled td: %d\r\n", scheduled_td->id );
    kernel_exit(scheduled_td->retval, scheduled_td->sp, scheduled_td->spsr);

    asm volatile(
     "mov %0, r1\n\t" 
     "mov %1, r2\n\t"
    : "+r"(new_sp_reg), "+r"(new_spsr_reg)
    );

    new_sp = new_sp_reg;
    scheduled_td->sp = new_sp;
    new_spsr = new_spsr_reg;
    scheduled_td->spsr = new_spsr;

    add_to_priority( &gc, scheduled_td->priority, scheduled_td );

    // schedule -> activate -> handle
  }

  bwprintf(COM2, "\n\rGoodbye\n\r");

  return 0;
}



