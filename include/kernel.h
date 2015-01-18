#ifndef __KERNEL_H__
#define __KERNEL_H__

#include "task_descriptor.h"

typedef struct global_context_t {
  
  task_descriptor_t tds[TD_MAX];
  task_descriptor_t * td_free;
  //static task_descriptor_t * td_busy;
  unsigned int td_free_num;//: 10;// for debugging
  unsigned int td_glb_id;
  unsigned int user_space[TD_MAX * TD_SIZE];

} global_context_t;

#endif
