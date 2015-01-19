#ifndef __KERNEL_H__
#define __KERNEL_H__

#include "task_descriptor.h"

#define TD_MAX 250
#define TD_SIZE 0x3000
#define USER_SPACE_SIZE TD_MAX * TD_SIZE
#define REDBOOT_OFFSET 0x218000
#define NULL ((void *) 0)

// "Global" variables used by the kernel
typedef struct global_context_t {
  task_descriptor_t tds[TD_MAX];
  task_descriptor_t * td_first_free;
  task_descriptor_t * td_last_free;
  task_descriptor_t * cur_task;
  unsigned int td_free_num;
  unsigned int td_glb_id;
  unsigned int user_space[TD_MAX * TD_SIZE];

} global_context_t;

void kernel_exit(int retval, unsigned int *sp, unsigned int spsr);

void init_kernelentry();

#endif
