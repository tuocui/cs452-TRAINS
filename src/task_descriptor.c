#include <tools.h>
#include <task_descriptor.h>
#include <kernel.h>

void tds_init(global_context_t *gc) {
  gc->td_first_free = gc->tds;
  gc->td_last_free = &(gc->tds[TD_MAX-1]);
  gc->td_free_num = TD_MAX;
  gc->td_glb_id = 1; 
  task_descriptor_t *cur_td;  

  int i = 0;
  for( ; i < TD_MAX; ++i) {
    cur_td = &(gc->tds[i]);
    cur_td->sp = gc->user_space + (TD_SIZE * (i + 1)) - 1;
    cur_td->orig_sp = cur_td->sp;
    cur_td->spsr = 0xd0;
    cur_td->retval = 0;
    cur_td->id = 0;
    cur_td->parent = NULL;
    cur_td->priority = 0; // no priority
    cur_td->status = TD_ZOMBIE;
    cur_td->next_free = &(gc->tds[i + 1]);
    cur_td->next_in_priority = NULL;
  }
  
  ((gc->tds)[TD_MAX - 1]).next_free = NULL;
}

task_descriptor_t * tds_create_td(global_context_t *gc, unsigned int priority, int *code) { 
  if(gc->td_first_free == NULL) {
    assert(gc->td_free_num == 0);
    return NULL;
  }

  task_descriptor_t * td_out = gc->td_first_free;

  td_out->id = gc->td_glb_id;
  ++(gc->td_glb_id);
  td_out->parent = gc->cur_task;
  td_out->priority = priority;
  td_out->status = TD_READY;

  /* remove from free list */
  gc->td_first_free = td_out->next_free;
  td_out->next_free = NULL;

  // Set initial pc;
  *(td_out->sp - 10) = (int)code;
  *(td_out->sp - 2) = (unsigned int)(td_out->sp);
  td_out->sp -= 10;

  /* insanity check */
  --(gc->td_free_num);
  assert(gc->td_free_num >= 0);

  return td_out;
}

void tds_remove_td(global_context_t *gc, task_descriptor_t * td) {
  /* we keep its stack pointer only */
  td->spsr = 0xd0;
  td->sp = td->orig_sp;
  td->id = 0;
  td->parent = NULL;
  td->priority = 0;
  td->status=TD_ZOMBIE;

  /* add to free list */
  (gc->td_last_free)->next_free = td;
  td->next_free = NULL;
  td->next_in_priority = NULL;
  gc->td_last_free = td;
  ++(gc->td_free_num);

}

