#include <tools.h>
#include <kernel.h>

void handle_send( global_context_t *gc ) {
  register unsigned int int_reg asm("r0");
  register char *      char_reg asm("r1");
  register unsigned int *cur_sp_reg asm("r2") = (gc->cur_task)->sp;
  unsigned int          *cur_sp;
  int tid, msglen, replylen;
  char *msg, *reply;
  
  //debug("cur_sp_reg: %x", cur_sp_reg);

  /* get the first two arguments: tid and *msg */
  asm volatile( 
      "msr cpsr_c, #0xdf\n\t"       // switch to system mode
      "mov r3, %2\n\t"              // load user sp to r3
      "add r3, r3, #44\n\t"         // 10 registers + pc saved
      "ldmfd r3!, {%0, %1}\n\t"     // store two args
      "mov %2, r3\n\t"              // update local cur_sp_reg 
      "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
      : "+r"(int_reg), "+r"(char_reg), "+r"(cur_sp_reg)
      );
   cur_sp = cur_sp_reg;
   tid = int_reg;
   msg = char_reg;

  /* get the next two arguments: msglen and *reply */
  cur_sp_reg = cur_sp;
  asm volatile( 
      "msr cpsr_c, #0xdf\n\t"       // switch to system mode
      "mov r3, %2\n\t"              // load user sp to r3
      "ldmfd r3!, {%0, %1}\n\t"     // store two args
      "mov %2, r3\n\t"              // update local cur_sp_reg 
      "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
      : "+r"(int_reg), "+r"(char_reg), "+r"(cur_sp_reg)
      );
   cur_sp = cur_sp_reg;
   msglen = int_reg;
   reply = char_reg;

   /* get the last argument: replylen */
   cur_sp_reg = cur_sp;
   asm volatile( 
      "msr cpsr_c, #0xdf\n\t"       // switch to system mode
      "mov r3, %1\n\t"              // load user sp to r3
      "ldmfd r3!, {%0}\n\t"         // store one arg
      "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
      : "+r"(int_reg), "+r"(cur_sp_reg)
      );
   replylen = int_reg;

   debug("tid: %d, msg: %x, msglen: %d, reply: %x, replylen: %d",
       tid, msg, msglen, reply, replylen);

   /* TODO: if task_id's index is not valid , return -1 */

   /* TODO: if task_id has exited, return -2 */

   /* now we know the receiver is alive */

   /* TODO: if receiver is SEND_BLOCK (waiting), 
    *       1. copy the message to receiver's user space,
    *       2. change receiver's state to READY, set its return val etc.
    *       3. schedule receiver
    *       4. change sender's state to REPLY_BLOCK
    */
   
   /* TODO: if receiver is not waiting,
    *       1. find receiver's TD and put sender in the end of its sender's queue
    *       2. change sender's state to RECEIVE_BLOCK
    *       3. ?? something else ?? 
    */

   /* now the sender is either RECEIVE_BLOCK or REPLY_BLOCK, 
    * the RECEIVER will wake it up by looking up the sender's queue,
    * or the replyer will wake it up by specifying its index 
    */
}



void handle_create( global_context_t *gc ) {
  register unsigned int priority_reg asm("r0");
  unsigned int priority;
  register void (*code_reg)() asm("r1");
  void (*code)();
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
  
  /*  check priority */
  if(priority > PRIORITY_MAX) {
    gc->cur_task->retval = -1;
  } 
  else {
    task_descriptor_t *new_td = tds_create_td(gc, priority, (int)(code));

    /* check if there are tds available */
    if(new_td == NULL) {
      gc->cur_task->retval = -2;
    }
    else {
      (gc->cur_task)->retval = new_td->id;
      add_to_priority( gc, new_td );
    }
  }
  /* add current task back to queue regardless */
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
