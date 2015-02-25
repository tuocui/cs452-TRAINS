#include <tools.h>
#include <kernel.h>

void tds_init(global_context_t *gc) {
  int i = 0;
  ///* loading bar */ 
  //// TODO: should combine it with screen layout in the future
  //int bar_size = 40;
  //int block_size = USER_SPACE_SIZE / bar_size;
  //int block_step = 0;
  //int blocks = 1;
  ///* loading bar end */

  gc->td_first_free = gc->tds;
  gc->td_last_free = &(gc->tds[TD_MAX-1]);
  gc->td_free_num = TD_MAX;
  gc->td_magic = TD_MAGIC;

  task_descriptor_t *cur_td;  

    for( ; i < USER_SPACE_SIZE ; ++i ) {
      (gc->user_space)[i] = 0;
      //debug("init i: %d", i);

    ///* loading bar */
    //if(block_step == block_size) {
    //  bwprintf(COM2, "\033[s\033[50D%d%%\033[u\033[44m ", 
    //                 blocks * 100 / bar_size );
    //  ++blocks;
    //  block_step = 0;
    //}
    //++block_step;
    ///* loading bar end */
  }

  ///* loading bar */
  //bwprintf(COM2, "\033[40m\n\r");
  ///* loading bar end */

  i = 0;
  for( ; i < TD_MAX; ++i) {
    cur_td = &(gc->tds[i]);
    cur_td->sp = gc->user_space + (TD_SIZE * (i + 1)) - 1;
    cur_td->orig_sp = cur_td->sp;
  *(cur_td->orig_sp - (TD_SIZE - 1)) = gc->td_magic; // set magic number
    cur_td->spsr = 0x10;
    cur_td->retval = 0;
    cur_td->id = i;
    cur_td->parent_id = 0;
    cur_td->priority = 0; // no priority
    cur_td->status = TD_ZOMBIE;
    cur_td->next_free = &(gc->tds[i + 1]);
    cur_td->next_in_priority = NULL;
    cur_td->first_sender_in_queue = NULL;
    cur_td->last_sender_in_queue = NULL;
    cur_td->next_sender = NULL;
  }
  
  ((gc->tds)[TD_MAX - 1]).next_free = NULL;
}

task_descriptor_t * tds_create_td(global_context_t *gc, unsigned int priority, int code_addr) { 
  if(gc->td_first_free == NULL) {
    assert(0, gc->td_free_num == 0);
    return NULL;
  }

  task_descriptor_t * td_out = gc->td_first_free;

  td_out->id = (td_out->id & (TD_MAX - 1)) | (((td_out->id >> TD_BIT) + 1) << TD_BIT) ;
  td_out->parent_id = gc->cur_task == NULL ? 0 : (gc->cur_task)->id;
  td_out->priority = priority;
  td_out->status = TD_READY;

  /* remove from free list */
  gc->td_first_free = td_out->next_free;
  td_out->next_free = NULL;

  // Set initial pc;
  *(td_out->sp - 13) = code_addr + REDBOOT_OFFSET;
  *(td_out->sp - 2) = (unsigned int)(td_out->sp);
  td_out->sp -= 13;

  /* insanity check */
  --(gc->td_free_num);
  assert(0, gc->td_free_num >= 0);

  return td_out;
}

void tds_remove_td(global_context_t *gc, task_descriptor_t * td) {
  /* we keep its stack pointer only */
  td->spsr = 0xd0;
  td->sp = td->orig_sp;
  td->id = 0;
  td->parent_id = 0;
  td->priority = 0;
  td->status=TD_ZOMBIE;

  /* add to free list */
  (gc->td_last_free)->next_free = td;
  td->next_free = NULL;
  td->next_in_priority = NULL;
  gc->td_last_free = td;
  ++(gc->td_free_num);
}

