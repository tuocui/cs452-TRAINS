#include <tools.h>
#include <kernel_syscall.h>
#include <kernel.h>

void handle_create( global_context_t *gc ) {
  register unsigned int priority_reg asm("r0");
  unsigned int priority;
  register int *code_reg asm("r1");
  int *code;
  register unsigned int *cur_sp_reg asm("r3") = (gc->cur_task)->sp;

  asm volatile(
    "msr cpsr_c, #0xdf\n\t"
    "mov r1, %2\n\t"
    "add r2, r1, #44\n\t" // 10 registers + pc saved
    "ldmfd r2, {%0, %1}\n\t"
    "msr cpsr_c, #0xd3\n\t"
    : "+r"(priority_reg), "+r"(code_reg), "+r"(cur_sp_reg)
  );
  priority = priority_reg;
  code = code_reg;
  task_descriptor_t *new_td = tds_create_td(gc, priority, (int*)((int) code + REDBOOT_OFFSET));
  (gc->cur_task)->retval = new_td->id;
  add_to_priority( gc, new_td );
  add_to_priority( gc, gc->cur_task );
}

void handle_pass( global_context_t *gc ) {
  add_to_priority( gc, gc->cur_task );
}

void handle_kill( global_context_t *gc ) {
  // Not much to do, make sure to remove from ALL queues
  // including message queues

  // NOTE: This task is already removed from priority queue.
  // Since kill does NOT reclaim memory, we don't need to add it back
  // to the free list

  // Just change status, lol.
  (gc->cur_task)->status = TD_ZOMBIE;
}

void handle_my_tid( global_context_t *gc ){
  task_descriptor_t *td = gc->cur_task;
  td->retval = td->id;
  add_to_priority( gc, gc->cur_task );
}

void handle_my_parent_tid( global_context_t *gc ) {
  task_descriptor_t *td = gc->cur_task;
  td->retval = td->parent_id;
  add_to_priority( gc, gc->cur_task );
}
