//For Testing
#include <stdio.h>
#include <assert.h>

//#include "task_descriptor.h"
#define TD_MAX 50
#define TD_SIZE 0x3000
#define USER_SPACE_SIZE TD_MAX * TD_SIZE
#define REDBOOT_OFFSET 0x218000


//TODO: put this in .h
//TODO: low priority :macro functions to assert each task's sp boundary
typedef struct task_descriptor_t{
  unsigned int *sp; 
  unsigned int spsr;

  unsigned int id;
  unsigned int parent_id;
  unsigned int priority;//;: 4;

  enum { 
    TD_READY,
    TD_RECEIVE_BLOCKED,
    TD_REPLY_BLOCKED,
    TD_SEND_BlOCKED,
    TD_ZOMBIE,
  } status;//: 8;

//  struct task_descriptor_t * prev;
  struct task_descriptor_t * next_free;

} task_descriptor_t ;


static unsigned int checker;

/* Things TODO: 
   on initialization:
    1. allocate user space for all tasks 
    2. assign a piece of memory for each task
    3. assign mode to zombie
    4. initialize static variables
   on tds_create_td: 
    1. assign a glb id when a td is given to the scheduler
    2. change mode to ready
    3. set *(sp-10) = pc; *(sp-2) = sp, sp -= 10;  
 */


void tds_init() {
  td_glb_id = 1; 
  td_free = tds;
//  td_busy = NULL;
  td_free_num = TD_MAX;
  
  int i = 0;
  for( ; i < TD_MAX; ++i) {
    task_descriptor * td = &(tds[i]);
    td->spsr = #0xd0;
    td->sp = REDBOOT_OFFSET + user_space + TD_SIZE * (i + 1) - 1;
    td->id = 0;
    td->parent_id = 0;
    td->priority = 0; // no priority
    td->status = TD_ZOMBIE;
    //td->prev = &(tds[i - 1]);
    td->next_free = &(tds[i + 1]);
  }
  
  //tds[0]->prev_td = NULL;
  tds[TD_MAX - 1]->next_free_td = NULL;
}

task_descriptor_t * tds_create_td(unsigned int priority, (void*) code) { 
  if(td_free == NULL) {
    assert(td_free_num == 0);

    return NULL;
  }

  assert(td_free->prev == NULL);

  task_descriptor_t * td_out = td_free;

  /* remove from free list */
  td_free = td_out->td_next_free;
  if(td_free != NULL)
    td_free->prev = NULL;
  
//  /* add to busy list */
//  td_out->next_free = td_busy;
//  if(td_busy != NULL)
//    td_busy->prev = td_out;
//
//  td_out->prev = NULL;
//  td_busy = td_out;
  
  /* insanity check */
  --td_free_num;
  assert(td_free_num >= 0);
  return td_out;
}

void tds_remove_td(task_descriptor_t * td) {
  /* we keep its stack pointer only */
  td->pc = 0;
  td->spsr = 0;

  td->id = 0;
  td->parent_id = 0;
  td->priority = 0;
  td->status=TD_ZOMBIE;

  /* remove from busy list */
  if(td->prev != NULL) 
    td->prev->next_free = td->next_free;
  else 
    td_busy = td->next_free;

  if(td->next_free != NULL)
    td-next_free->prev = td->prev;

  /* add to free list */
  td->prev = NULL;
  td->next_free = td_free;

  if(td_free != NULL)
    td_free->prev = td;

  td_free = td;

  /* insanity check */
  ++td_free_num;
  if(td->busy == NULL)
    assert(td_free_num == TD_MAX);

}
