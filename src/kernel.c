#include <tools.h>
#include <kernel.h>

void kernel_init( global_context_t *gc) {
  gc->cur_task = NULL;
  gc->priority_bitmap = 0;

  tds_init(gc);
  //TODO: find out why init_regs is not linked, and how
  // init_kernelentry is linked even though it's commented out
  // in .h 
  //init_regs();
  init_kernelentry();
  init_schedulers(gc);
}

int activate( global_context_t *gc, task_descriptor_t *td ) {
  register int request_type_reg asm("r0"); // absolute 
  int request_type;
  register unsigned int *new_sp_reg asm("r1"); // absolute 
  unsigned int *new_sp;
  register unsigned int new_spsr_reg asm("r2"); // absolute 
  unsigned int new_spsr;
  gc->cur_task = td;

  kernel_exit(td->retval, td->sp, td->spsr);

  asm volatile(
    "mov %0, r0\n\t"
    "mov %1, r1\n\t"
    "mov %2, r2\n\t"
  : "+r"(request_type_reg), "+r"(new_sp_reg), "+r"(new_spsr_reg)
  );

  /* check td magic, failures indicate user stack is too small */
  assert(*(td->orig_sp - (TD_SIZE - 1)) == gc->td_magic);

  request_type = request_type_reg;
  new_sp = new_sp_reg;
  td->sp = new_sp;
  new_spsr = new_spsr_reg;
  td->spsr = new_spsr;
  return request_type;
}

void handle( global_context_t *gc, int request_type ) {
  switch( request_type ) {
  case SYS_CREATE:
    handle_create( gc );
    break;
  case SYS_MY_TID:
    handle_my_tid( gc );
    break;
  case SYS_MY_PARENT_TID:
    handle_my_parent_tid( gc );
    break;
  case SYS_PASS:
    handle_pass( gc );
    break;
  case SYS_KILL:
    handle_kill( gc );
    break;
  default:
    break;
  }
}


int main(int argc, char *argv[]) {

  int request_type;
  bwputstr( COM2, "LOADING... WE ARE FASTER THAN WINDOWS :)\r\n" );
  global_context_t gc;
  kernel_init( &gc );
  bwputstr( COM2, "FINISHED INITIALIZATION. WOO!\r\n" );

  task_descriptor_t *first_td = tds_create_td(&gc, 5, (int*)( (int) &first_user_task + REDBOOT_OFFSET));
  add_to_priority( &gc, first_td );

  while (1){
    task_descriptor_t *scheduled_td = schedule( &gc );
    if ( scheduled_td == NULL ) {
      // HHEYYYYYYY, we exit
      break;    
    }

    request_type = activate( &gc, scheduled_td );
    handle( &gc, request_type );
  }

  bwprintf(COM2, "\n\rGoodbye\n\r");

  return 0;
}



//TODO: delete test once we are assured the id calculation is coorect
//test for id generating
//void test() {
//  int bit = 7;
//  int tds = (1 << (bit+1)) - 1;
//  int id = 0;
//  int i = 0;
//  int j = 0;
//  int id_arr[tds];
//  // initialize id_arr
//  
//  for(id = 0; id < tds; ++id) {
//    id_arr[id] = id+1;
//  }
//
//  for(j = 0; j < 5; ++j) {
//
//    for(i = 0; i < tds; ++i) {
//      id_arr[i] = (id_arr[i] & tds) | (((id_arr[i] >> bit) + 1) << bit) ;
//      int gen = id_arr[i] >> bit;
//      int idx = id_arr[i] & tds;
//      debug("gen: %d, idx: %d", gen, idx);
//    }
//
//  }
//}


