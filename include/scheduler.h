#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

struct global_context_t;
struct task_descriptor_t;

typedef struct scheduler_t{
  enum {
    HIGH=1,
    MEDIUM,
    LOW,
    IDLE,
  } priority;

  unsigned int num_in_queue;
  task_descriptor_t *first_in_queue;
  task_descriptor_t *last_in_queue;
} scheduler_t;

void init_schedulers( struct global_context_t* gc );

task_descriptor_t *schedule( struct global_context_t *gc);

void add_to_priority( struct global_context_t *gc, unsigned int priority, struct task_descriptor_t *td );

int get_highest_priority( struct global_context_t *gc );

#endif
