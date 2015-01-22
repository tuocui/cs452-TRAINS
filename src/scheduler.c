#include <tools.h>
#include <kernel.h>

int get_highest_priority( global_context_t *gc ) {
  // Thanks Wikipedia
  unsigned int bm = gc->priority_bitmap;
  int rtn;
  rtn = gc->de_bruijn_bit_positions[(((bm & -bm) * 0x077CB531U)) >> 27];
  return rtn;
}

void init_schedulers( global_context_t* gc ) {
  int i = 0;
  for ( ; i <= PRIORITY_MAX; ++i ) {
    scheduler_t *scheduler = &((gc->priorities)[i]);
    scheduler->priority = i;
    scheduler->num_in_queue = 0;
    scheduler->first_in_queue = NULL;
    scheduler->last_in_queue = NULL;
  }
  gc->priority_bitmap = 0;
  (gc->de_bruijn_bit_positions)[0] = 0;
  (gc->de_bruijn_bit_positions)[1] = 1;
  (gc->de_bruijn_bit_positions)[2] = 28;
  (gc->de_bruijn_bit_positions)[3] = 2;
  (gc->de_bruijn_bit_positions)[4] = 29;
  (gc->de_bruijn_bit_positions)[5] = 14;
  (gc->de_bruijn_bit_positions)[6] = 24;
  (gc->de_bruijn_bit_positions)[7] = 3;
  (gc->de_bruijn_bit_positions)[8] = 30;
  (gc->de_bruijn_bit_positions)[9] = 22;
  (gc->de_bruijn_bit_positions)[10] = 20;
  (gc->de_bruijn_bit_positions)[11] = 15;
  (gc->de_bruijn_bit_positions)[12] = 25;
  (gc->de_bruijn_bit_positions)[13] = 17;
  (gc->de_bruijn_bit_positions)[14] = 4;
  (gc->de_bruijn_bit_positions)[15] = 8;
  (gc->de_bruijn_bit_positions)[16] = 31;
  (gc->de_bruijn_bit_positions)[17] = 27;
  (gc->de_bruijn_bit_positions)[18] = 13;
  (gc->de_bruijn_bit_positions)[19] = 23;
  (gc->de_bruijn_bit_positions)[20] = 21;
  (gc->de_bruijn_bit_positions)[21] = 19;
  (gc->de_bruijn_bit_positions)[22] = 16;
  (gc->de_bruijn_bit_positions)[23] = 7;
  (gc->de_bruijn_bit_positions)[24] = 26;
  (gc->de_bruijn_bit_positions)[25] = 12;
  (gc->de_bruijn_bit_positions)[26] = 18;
  (gc->de_bruijn_bit_positions)[27] = 6;
  (gc->de_bruijn_bit_positions)[28] = 11;
  (gc->de_bruijn_bit_positions)[29] = 5;
  (gc->de_bruijn_bit_positions)[30] = 10;
  (gc->de_bruijn_bit_positions)[31] = 9;

}

void add_to_priority( global_context_t *gc, task_descriptor_t *td ) {
  unsigned int priority = td->priority;
  assert(priority != 0, "ERROR: should not pass in priority = 0");
  scheduler_t *scheduler = &((gc->priorities)[priority]);
  task_descriptor_t *last_td = scheduler->last_in_queue;
  if ( last_td != NULL ) {
    last_td->next_in_priority = td;
  }
  if ( scheduler->first_in_queue == NULL ) {
    scheduler->first_in_queue = td;  
  }

  td->status = TD_READY;
  gc->priority_bitmap = gc->priority_bitmap | ( 1 << priority );
  scheduler->last_in_queue = td;  

  ++(scheduler->num_in_queue);

  assert(scheduler->num_in_queue > 0);
  assert(scheduler->num_in_queue <= TD_MAX);
}

task_descriptor_t *schedule( global_context_t *gc) {
  int highest_priority = get_highest_priority( gc );
  if ( highest_priority == 0 ) {
    return NULL;  
  }
  scheduler_t *scheduler = &((gc->priorities)[highest_priority]);
  
  task_descriptor_t *td = scheduler->first_in_queue;
  scheduler->first_in_queue = td->next_in_priority;
  if (scheduler->first_in_queue == NULL) {
    scheduler->last_in_queue = NULL;
    gc->priority_bitmap = gc->priority_bitmap ^ ( 1 << highest_priority );
  }
  td->next_in_priority = NULL;
  --(scheduler->num_in_queue);
  
  assert(scheduler->num_in_queue >= 0);

  return td;
}
