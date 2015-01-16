//For Testing
#include <stdio.h>
#include <assert.h>

#define TD_MAX 50

//TODO: put this in .h
typedef struct {
  unsigned int id;
  unsigned int parent_id;
  unsigned int spsr;
  unsigned int *sp; 
  unsigned int priority: 4;

  task_descriptor_t * prev;
  task_descriptor_t * next;

  enum { 
    TD_READY,
    TD_RECEIVE_BLOCKED,
    TD_REPLY_BLOCKED,
    TD_SEND_BlOCKED,
    TD_ZOMBIE,
  } status: 8;

} task_descriptor_t;


static task_descriptor_t td_arr[TD_MAX];
static task_descriptor_t * td_free;
static task_descriptor_t * td_busy;
static unsigned int td_free_num: 10;// for debugging
static unsigned int td_glb_id;

/* we only zero out the fields in init(), values are assigned when
   give it to the scheduler (or by the scheduler)
 */


void td_arr_init() {
  td_glb_id = 1; 
  td_free = td_arr;
  td_busy = NULL;
  td_free_num = TD_MAX;
  
  int i = 0;
  for( ; i < TD_MAX, ++i) {
    task_descriptor * td = &(td_arr[i]);
    td->id = 0;
    td->parent_id = 0;
    td->spsr = 0;
    td->sp = 0;
    td->priority = 0;
    td->status = TD_ZOMBIE;
    *td->prev = td_arr[i - 1];
    *td->next = td_arr[i + 1];
  }
  
  td_arr[0]->prev_td = NULL;
  td_arr[TD_MAX - 1]->next_td = NULL;
}

task_descriptor_t * td_arr_get() { 
  if(td_free == NULL) {
    assert(td_free_num == 0);
    return NULL;
  }

  task_descriptor_t * td_out = td_free;
  td_free = td_out->td_next;
  
  *td_out->next = (td_busy == NULL) ? NULL : *td_busy->next;
  *td_out->prev = NULL;
  td_busy = td_out;
  
  --td_free_num;

  assert(td_free_num >= 0);
  return td_out;
}

void td_arr_destroy(task_descriptor_t * td) {
  td->id = 0;
  td->parent_id = 0;
  td->spsr = 0;
  td->sp = 0;
  td->priority = 0;

  *td->prev = NULL;
  *td->next = td_free;

  if(td_free != NULL)
    *td_free->prev = td;

  td_free = td;

  ++td_free_num;

}
