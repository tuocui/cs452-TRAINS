#ifndef __KERNEL_H__
#define __KERNEL_H__

#include "ts7200.h"

#include "global.h"
#include "task_descriptor.h"
#include "scheduler.h"
#include "kernel_syscall.h"
#include "user_task.h"
#include "context_switch.h"

#define TD_SIZE 0x3000
#define TD_MAGIC 0x3b1a4ef5
#define HWI_MAGIC 0xab
#define USER_SPACE_SIZE (TD_MAX * TD_SIZE)


/* interrupts */
#define TIMER3_INT         51
#define UART1_COMBINED_INT 52
#define UART2_COMBINED_INT 54

/* IO status offsets */
#define COM1_TRANSMIT_MASK 0x1
#define COM1_CTS_MASK      0x2
#define COM1_RECEIVE_MASK  0x4

// "Global" variables used by the kernel
typedef struct global_context_t {

  /* task descriptor */
  task_descriptor_t * cur_task;
  task_descriptor_t tds[TD_MAX];
  task_descriptor_t * td_first_free;
  task_descriptor_t * td_last_free;
  unsigned int td_magic;
  unsigned int td_free_num;
  unsigned int user_space[TD_MAX * TD_SIZE];
  int num_tasks;

  /* scheduler */
  scheduler_t priorities[PRIORITY_MAX + 1];
  unsigned int priority_bitmap;
  int de_bruijn_bit_positions[32];

  task_descriptor_t *interrupts[NUM_INTS];
  unsigned int num_missed_clock_cycles;
  int com1_status;
  int num_ticks;
  int num_ticks_idle;
  //int com2_status;

} global_context_t;

int get_lowest_set_bit( global_context_t *gc, int bm );

void kernel_init( struct global_context_t *gc);

int activate( global_context_t *gc, task_descriptor_t *td );

void handle( global_context_t *gc, int request_type );

inline void clean_set_bit( int * bm, int offset ); 


#endif
