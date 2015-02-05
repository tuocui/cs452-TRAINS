#include "tools.h"
#include "kernel.h"


void cache_init( ) {
  asm volatile( 
    "MRC p15, 0, r0, c1, c0, 0\n\t" // read cp15 to r0
    "ORR r0, r0, #0x5\n\t"         //, MMU, D-cache
    "ORR r0, r0, #0x1000\n\t"         //, MMU, D-cache and I-cache
    "MCR p15, 0, r0, c1, c0, 0\n\t" // store the new value
  ); 
}

void hwi_cleanup( ) {
  /* clear all interrupt bits */
  *((unsigned int *)(VIC1_BASE + VICX_INT_ENCLEAR_OFFSET)) = CLEAR_ALL;
  *((unsigned int *)(VIC2_BASE + VICX_INT_ENCLEAR_OFFSET)) = CLEAR_ALL;
  /* Clear timer interrupt bit */
  int *timer_base = (int *)(TIMER3_BASE + CLR_OFFSET);
  *timer_base = 1;
}

void hwi_init( ) {
  /* turn on timer3 interrupt, select as IRQ*/
  *((unsigned int *)(VIC2_BASE + VICX_INT_SELECT_OFFSET)) &= TIMER3_IRQ_MASK;
  *((unsigned int *)(VIC2_BASE + VICX_INT_ENABLE_OFFSET)) |= TIMER3_INT_ON;
}


void kernel_init( global_context_t *gc) {
  gc->cur_task = NULL;
  gc->priority_bitmap = 0;

  hwi_cleanup( );
  hwi_init( );
  
  init_kernelentry();
#ifdef CACHE
  bwprintf( COM2, "TURNING ON CACHE ...\n\r " );
  cache_init( );
#endif

  tds_init(gc);
  init_schedulers(gc);
}

/* TODO: Move this to helper file */
int get_lowest_set_bit( global_context_t *gc, int bm ) {
  // Thanks Wikipedia
  int rtn;
  rtn = gc->de_bruijn_bit_positions[(((bm & -bm) * 0x077CB531U)) >> 27];
  return rtn;
}

inline void clean_set_bit( int *bm , int offset ) {
  *bm = *bm ^ ( 1 << offset );
}

int activate( global_context_t *gc, task_descriptor_t *td ) {
  register int          request_type_reg  asm("r0"); // absolute 
  register unsigned int *new_sp_reg       asm("r1"); // absolute 
  register unsigned int new_spsr_reg      asm("r2"); // absolute 
  register int          user_r0_reg       asm("r3"); // absolute
  int                   request_type;
  int                   hwi_request_flag = -1;
  gc->cur_task = td;
  kernel_exit(td->retval, td->sp, td->spsr, &hwi_request_flag);

  asm volatile(
    "mov %0, r0\n\t"
    "mov %1, r1\n\t"
    "mov %2, r2\n\t"
    "mov %3, r3\n\t"
  : "+r"(request_type_reg), "+r"(new_sp_reg), 
    "+r"(new_spsr_reg), "+r"(user_r0_reg)
  );

  request_type = request_type_reg;
  td->sp = new_sp_reg;
  td->spsr = new_spsr_reg;
  td->retval = user_r0_reg;

  //debug( "sp: %x", td->sp );
  assert( hwi_request_flag == HWI_MAGIC || hwi_request_flag == -1 );
  /* if hwi request type is set, we update request_type */
  if( hwi_request_flag == HWI_MAGIC) {
    request_type = HWI;
  }

  /* reset hwi request info */
  hwi_request_flag = -1;

  /* check td magic, failures indicate user stack is too small */
  assert(*(td->orig_sp - (TD_SIZE - 1)) == gc->td_magic);
  return request_type;
}

void handle( global_context_t *gc, int request_type ) {
  int hwi_type;
  int vic_src_num = 0;
  int vic_base; // make this a local copy so we can modify its value
  switch( request_type ) {
  case HWI:
    while( vic_src_num < 2 ) {
      vic_base = (int)(*((int*)(VIC1_BASE + vic_src_num * 0x10000)));
      while( ( hwi_type = get_lowest_set_bit( gc, vic_base ) ) ) {
        //debug("INSIDE WHILE");
        clean_set_bit( &vic_base, hwi_type );
        hwi_type += ( vic_src_num * 32 );
        handle_hwi( gc, hwi_type );
      }
      
      assert( vic_base == 0 );
      ++vic_src_num;

      /* should have reset vic register at this point */
    }

    break;
  case SYS_CREATE:
    handle_create( gc );
    break;
  case SYS_MY_TID:
    handle_my_tid( gc );
    break;
  case SYS_MY_PARENT_TID:
    handle_my_parent_tid( gc );
    break;
  case SYS_SEND:
    handle_send( gc );
    break;
  case SYS_RECEIVE:
    handle_receive( gc );
    break;
  case SYS_REPLY:
    handle_reply( gc );
    break;
  case SYS_PASS:
    handle_pass( gc );
    break;
  case SYS_EXIT:
    handle_exit( gc );
    break;
  default:
    break;
  }
}

int main(int argc, char *argv[]) {

  debug("TD_BIT: %d", TD_BIT);
  debug("TD_MAX: %d", TD_MAX);

  int request_type;
  bwputstr( COM2, "LOADING... WE ARE FASTER THAN WINDOWS :)\r\n" );
  global_context_t gc;
  kernel_init( &gc );

  task_descriptor_t *first_td = tds_create_td(&gc, 5, (int)&first_user_task);
  add_to_priority( &gc, first_td );

  bwputstr( COM2, "FINISHED INITIALIZATION. WOO!\r\n" );

  FOREVER {
    task_descriptor_t *scheduled_td = schedule( &gc );
    if ( scheduled_td == NULL ) {
      // HHEYYYYYYY, we exit
      break;    
    }

    //debug( "BEFORE activate" );
    request_type = activate( &gc, scheduled_td );
    handle( &gc, request_type );
  }

  /* remove this to screw up the next team, lol */
  hwi_cleanup( );
  bwprintf(COM2, "\n\rExit Main\n\r");

  return 0;
}


