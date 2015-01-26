#include <tools.h>
#include <kernel.h>

/* memcpy for message passing */
// TODO: Utilize our 10 spare registers to do memcpy.
// Way faster than copying 1 byte at a time
void kmemcpy( char * dst, const char * src, size_t len ) {
  dst += len; 
  src += len;
  while(len-- > 0)  
    *--dst = *--src;
}

void handle_send( global_context_t *gc ) {
  register char         *char_reg   asm("r0");
  register unsigned int  int_reg    asm("r1");
  register unsigned int *pint_reg   asm("r2");
  register unsigned int *cur_sp_reg asm("r3") = (gc->cur_task)->sp;

  int tid_s, msglen_s, replylen_s;
  char *msg_s, *reply_s;
  
  //debug("cur_sp_reg: %x", cur_sp_reg);

  /* get the first argument: tid */
  asm volatile( 
    "msr cpsr_c, #0xdf\n\t"       // switch to system mode
    "add r3, %1, #44\n\t"         // 10 registers + pc saved
    "ldmfd r3!, {%0}\n\t"         // load tid arg
    "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
    : "+r"(int_reg), "+r"(cur_sp_reg)
  );
  tid_s = int_reg;

  /* get the next two arguments: *msg, and msglen */
  asm volatile( 
    "msr cpsr_c, #0xdf\n\t"       // switch to system mode
    //"add r3, %2, #48\n\t"         // 10 registers + pc saved
    "ldmfd r3!, {%0}\n\t"         // store two args
    "ldmfd r3!, {%1}\n\t"         // store two args
    "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
    : "+r"(char_reg), "+r"(int_reg), "+r"(cur_sp_reg)
  );
  msg_s = char_reg;
  msglen_s = int_reg;

  /* get the last arguments: *reply, replylen */
  asm volatile( 
    "msr cpsr_c, #0xdf\n\t"       // switch to system mode
    "ldmfd r3!, {%0, %1}\n\t"     // store two args
    "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
    : "+r"(char_reg), "+r"(int_reg), "+r"(cur_sp_reg)
  );
  reply_s = char_reg;
  replylen_s = int_reg;

  debug("SENDER: tid: %d, msg: %x, msglen: %d, reply: %x, replylen: %d",
    tid_s, msg_s, msglen_s, reply_s, replylen_s);

  /* if task_id's index is not valid , return -1 */
  // all id_idx are possible in this kernel
  // If generation == 0, then impossible
  int tid_idx_s = TID_IDX( tid_s );
  assert( tid_idx_s >= 0 );
  assert( tid_idx_s <= ( TD_MAX - 1 ));

  if( TID_GEN(tid_s) == 0 ){
    debug( "heeyyyyy, invalid id" );
    gc->cur_task->retval = -1;
    add_to_priority( gc, gc->cur_task );
    return;
  }

  /* if task has a different gen, or it's a zombie, return -2, schedule sender */
  task_descriptor_t * td_r = &(gc->tds[TID_IDX(tid_s)]);
  if( TID_GEN(td_r->id) != TID_GEN(tid_s) ||
    td_r->status == TD_ZOMBIE ) {
    gc->cur_task->retval = -2; 
    add_to_priority( gc, gc->cur_task );
    return;
  }

  /* now we know the receiver is exists */

  /* if receiver is SEND_BLOCK (waiting), 
  *   0. find out length of the data to transfer, should be the min of two
  *   1. copy the message to receiver's user space, set receiver's *tid
  *   2. change receiver's state to READY, set its return val, what else?
  *   3. wake up the receiver and schedule it
  *   4. change sender's state to REPLY_BLOCK
  */
  if( td_r->status == TD_SEND_BLOCKED ) {
    /* reuse previous registers */
    int_reg = 0;
    pint_reg = NULL;
    char_reg = NULL;
    register unsigned int *receiver_sp_reg asm("r3") = td_r->sp;
    unsigned int *ptid_r, msglen_r;
    char *msg_r;

  /* get receiver's arguments: *tid, *msg and msglen */
   asm volatile( 
      "msr cpsr_c, #0xdf\n\t"       // switch to system mode
      "add r3, %3, #44\n\t"         // 10 registers + pc saved
      "ldmfd r3!, {%0}\n\t"         // load *tid, must load separately here
      "ldmfd r3!, {%1}\n\t"         // load *msg
      "ldmfd r3!, {%2}\n\t"         // load msglen
      "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
      : "+r"(pint_reg), "+r"(char_reg), "+r"(int_reg), "+r"(receiver_sp_reg)
    );
    ptid_r = pint_reg;
    msg_r = char_reg;
    msglen_r = int_reg;

    debug("RECEIVER: tid_r: %x, msg_r: %x, msglen_r: %d", ptid_r, msg_r, msglen_r);

    *ptid_r = gc->cur_task->id;
    int msg_copied = min(msglen_s, msglen_r);
    kmemcpy( /*dst*/ msg_r, /*src*/ msg_s, msg_copied );

    td_r->retval = msg_copied;
    add_to_priority( gc, td_r );

    gc->cur_task->status = TD_REPLY_BLOCKED;
    return;
  }
  
  /* if receiver is not waiting,
   *       1. find receiver's TD and put sender in the end of its sender's queue
   *       2. change sender's state to RECEIVE_BLOCK
   *       3. ?? something else ?? 
   */
  if ( td_r->first_sender_in_queue == NULL ) {
    debug( "New Sender on Receive Queue" );
    td_r->first_sender_in_queue = gc->cur_task;
    td_r->last_sender_in_queue = gc->cur_task;
  } else {
    td_r->last_sender_in_queue->next_sender = gc->cur_task;
    td_r->last_sender_in_queue = gc->cur_task;
    gc->cur_task->next_sender = NULL;
  }
  gc->cur_task->status = TD_RECEIVE_BLOCKED;

 /* now the sender is either RECEIVE_BLOCK or REPLY_BLOCK, 
  * the RECEIVER will wake it up by looking up the sender's queue,
  * or the replyer will wake it up by specifying its index 
  */
}

void handle_receive( global_context_t *gc ) {
  /* Receive steps:
     1. Check if there's a sender TD on it's queue.
     2. If there is, then grab the sender information, and message and move to cur_task's stack.
     3. If not, simply set state to SEND_BLOCK
  */
  task_descriptor_t *r_td = gc->cur_task;
  if ( r_td->first_sender_in_queue == NULL ) {
    r_td->status = TD_SEND_BLOCKED;
    debug( "receive TD is now blocked" );
    // Anything else?
  } else {
    debug( "Receiving a send waiting in queue" );
    task_descriptor_t *s_td = r_td->first_sender_in_queue;
    assert(s_td->status == TD_RECEIVE_BLOCKED);
    r_td->first_sender_in_queue = s_td->next_sender;
    s_td->next_sender = NULL;
    if (r_td->first_sender_in_queue == NULL) {
      r_td->last_sender_in_queue = NULL;
    }

  register char         *char_reg   asm("r0");
  register unsigned int  int_reg    asm("r1");
  register unsigned int *pint_reg   asm("r2");
  register unsigned int *s_sp_reg   asm("r3") = s_td->sp;

  int tid_s, msglen_s, replylen_s;
  char *msg_s, *reply_s;

  /* get the first two arguments: tid and *msg */
  asm volatile( 
      "msr cpsr_c, #0xdf\n\t"       // switch to system mode
      "add r3, %1, #44\n\t"         // 10 registers + pc saved
      "ldmfd r3!, {%0}\n\t"         // store tid arg
      "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
      : "+r"(int_reg), "+r"(s_sp_reg)
      );
   tid_s = int_reg;

  /* get the next two arguments: *msg, and msglen */
  asm volatile( 
      "msr cpsr_c, #0xdf\n\t"       // switch to system mode
      "ldmfd r3!, {%0, %1}\n\t"     // store two args
      "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
      : "+r"(char_reg), "+r"(int_reg), "+r"(s_sp_reg)
      );
  msg_s = char_reg;
  msglen_s = int_reg;

  /* get the last arguments: *reply, replylen */
  asm volatile( 
     "msr cpsr_c, #0xdf\n\t"       // switch to system mode
     "ldmfd r3!, {%0, %1}\n\t"         // store two args
     "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
     : "+r"(char_reg), "+r"(int_reg), "+r"(s_sp_reg)
     );
  reply_s = char_reg;
  replylen_s = int_reg;

  debug("SENDER: tid: %d, msg: %x, msglen: %d, reply: %x, replylen: %d",
     tid_s, msg_s, msglen_s, reply_s, replylen_s);

  int_reg = 0;
  pint_reg = NULL;
  char_reg = NULL;
  register unsigned int *r_sp_reg asm("r3") = r_td->sp;
  unsigned int *ptid_r, msglen_r;
  char *msg_r;

  /* get receiver's arguments: *tid, *msg and msglen */
  asm volatile( 
      "msr cpsr_c, #0xdf\n\t"       // switch to system mode
      "add r3, %3, #44\n\t"         // 10 registers + pc saved
      "ldmfd r3!, {%0}\n\t"         // load *tid, must load separately here
      "ldmfd r3!, {%1}\n\t"         // load *msg
      "ldmfd r3!, {%2}\n\t"         // load msglen
      "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
      : "+r"(pint_reg), "+r"(char_reg), "+r"(int_reg), "+r"(r_sp_reg)
      );
   ptid_r = pint_reg;
   msg_r = char_reg;
   msglen_r = int_reg;

   debug("tid: %x, msg: %x, msglen: %d", ptid_r, msg_r, msglen_r);

   *ptid_r = s_td->id;
   int msg_copied = min(msglen_s, msglen_r);
   kmemcpy( /*dst*/ msg_r, /*src*/ msg_s, msg_copied );

   s_td->status = TD_REPLY_BLOCKED;

   r_td->retval = msg_copied;
   add_to_priority( gc, r_td );
  }
}

void handle_reply( global_context_t *gc ) {
  register char         *char_reg   asm("r0");
  register unsigned int  int_reg    asm("r1");
  register unsigned int *reply_sp_reg   asm("r3") = gc->cur_task->sp;

  int target_tid, replylen_r, replylen_s;
  char *reply_r, *reply_s;

  /* get replyer's arg: target tid */
  asm volatile( 
    "msr cpsr_c, #0xdf\n\t"       // switch to system mode
    "add r3, %1, #44\n\t"         // 10 registers + pc saved
    "ldmfd r3!, {%0}\n\t"         // load target tid arg
    "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
    : "+r"(int_reg), "+r"(reply_sp_reg)
  );
  target_tid = int_reg;
  
  //TODO: refactor: make check_tid func
  int tid_idx_s = TID_IDX( target_tid );
  assert( tid_idx_s >= 0 );
  assert( tid_idx_s <= ( TD_MAX - 1 ));

  if( TID_GEN(target_tid) == 0 ){
    debug( "heeyyyyy, invalid id" );
    gc->cur_task->retval = -1;
    add_to_priority( gc, gc->cur_task );
    return;
  }
  
  //TODO: refactor this, and define errno
  /* if task has a different gen, or it's a zombie, return -2, schedule sender */
  task_descriptor_t * target_td = &(gc->tds[tid_idx_s]);
  if( TID_GEN(target_td->id) != TID_GEN(target_tid) ||
    target_td->status == TD_ZOMBIE ) {
    gc->cur_task->retval = -2; 
    add_to_priority( gc, gc->cur_task );
    return;
  }

  /* now we know the sender exists 
   * TODO: 
   * 1. if the task is not REPLY_BLOCKED, return -3
   * 2. if sender's buffer_len < replylen, return -4
   * 3. else, cpy reply msg to sender's space, return 0
   */
  if( target_td->status != TD_REPLY_BLOCKED ) {
    gc->cur_task->retval = -3;
    add_to_priority( gc, gc->cur_task );
    return;
  }

  /* get the next two replyer's arguments: *reply, and replylen*/
  reply_sp_reg = gc->cur_task->sp;
  asm volatile( 
    "msr cpsr_c, #0xdf\n\t"       // switch to system mode
    "add r3, %2, #48\n\t"         // 10 registers + pc saved
    "ldmfd r3!, {%0, %1}\n\t"     // store two args
    "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
    : "+r"(char_reg), "+r"(int_reg), "+r"(reply_sp_reg)
  );
  reply_r = char_reg;
  replylen_r= int_reg;

  /* get the sender's args: *reply_s, replylen_s */
  char_reg = NULL;
  int_reg = 0;
  register unsigned int *send_sp_reg asm("r3") = target_td->sp; 
  asm volatile( 
    "msr cpsr_c, #0xdf\n\t"       // switch to system mode
    "add r3, %2, #56\n\t"         // 10 registers + pc saved
    "ldmfd r3!, {%0, %1}\n\t"     // store two args
    "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
    : "+r"(char_reg), "+r"(int_reg), "+r"(send_sp_reg)
   );
   reply_s = char_reg;
   replylen_s = int_reg;

   debug("reply_r: %x, replylen_r: %d\n\r reply_s: %x, replylen_s: %d",
       reply_r, replylen_r, reply_s, replylen_s);

   if( replylen_s < replylen_r ) {
      gc->cur_task->retval = -4;
      add_to_priority( gc, gc->cur_task );
      return;
   }

   /* now cpy msg over */
   kmemcpy( /*dst*/ reply_s, /*src*/ reply_r, replylen_r );

   target_td->retval = replylen_r;
   gc->cur_task->retval = 0;
   add_to_priority( gc, target_td );
   add_to_priority( gc, gc->cur_task );
    
}

void handle_create( global_context_t *gc ) {
  register unsigned int priority_reg asm("r0");
  unsigned int priority;
  register void (*code_reg)() asm("r1");
  void (*code)();
  register unsigned int *cur_sp_reg asm("r3") = (gc->cur_task)->sp;

  asm volatile(
    "msr cpsr_c, #0xdf\n\t"
    "add r3, %2, #44\n\t" // 10 registers + pc saved
    "ldmfd r3, {%0, %1}\n\t"
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
