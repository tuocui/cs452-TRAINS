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

int get_message_tid( unsigned int *sp, int offset ) {
#ifndef CLANG
  debug("sp, *sp: %x, %x", sp, *sp);
  debug("offset: %d", offset);
  register unsigned int  int_reg     asm("r1");
  register int           offset_reg  asm("r2") = offset;
  register unsigned int  *cur_sp_reg asm("r3") = sp;
  int rtn;
  /* get the first argument: tid */
  asm volatile( 
    "msr cpsr_c, #0xdf\n\t"       // switch to system mode
    "add r3, %1, %2\n\t"
    "ldmfd r3!, {%0}\n\t"         // load tid arg
    "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
    : "+r"(int_reg), "+r"(cur_sp_reg)
    : "r"(offset_reg)
  );
  rtn = int_reg;
  return rtn;
#endif
}

char *get_message( unsigned int *sp, int offset, int *msglen ) {
#ifndef CLANG
  /* get the next two arguments: *msg, and msglen */
  register char          *char_reg   asm("r0");
  register unsigned int  int_reg     asm("r1");
  register int           offset_reg  asm("r2") = offset;
  register unsigned int  *cur_sp_reg asm("r3") = sp;
  char *msg;
  asm volatile( 
    "msr cpsr_c, #0xdf\n\t"       // switch to system mode
    "add r3, %2, %3\n\t"
    "ldmfd r3!, {%0, %1}\n\t"     // store two args
    "msr cpsr_c, #0xd3\n\t"       // switch to svc mode
    : "+r"(char_reg), "+r"(int_reg), "+r"(cur_sp_reg)
    : "r"(offset_reg)
  );
  *msglen = int_reg;
  msg = char_reg;
  return msg;
#endif
}

// TODO: Errno's
int check_tid( global_context_t *gc, unsigned int tid ) {
  /* if task_id's index is not valid , return -1 */
  // all id_idx are possible in this kernel
  int tid_idx = TID_IDX( tid );
  assert( tid_idx >= 0 );
  assert( tid_idx <= ( TD_MAX - 1 ));

  // If generation == 0, then impossible
  if( TID_GEN(tid) == 0 ){
    //debug( "heeyyyyy, invalid id" );
    return -1;
  }

  /* if task has a different gen, or it's a zombie, return -2, schedule sender */
  task_descriptor_t * td = &(gc->tds[TID_IDX(tid)]);
  if( TID_GEN(td->id) != TID_GEN(tid) ||
    td->status == TD_ZOMBIE ) {
    return -2;
  }

  return 0;
}

void handle_send( global_context_t *gc ) {
  unsigned int tid_s;
  int msglen_s, replylen_s;
  char *msg_s, *reply_s;

  tid_s = get_message_tid( gc->cur_task->sp, 72 );
  // msglen_s set
  msg_s = get_message( gc->cur_task->sp, 56, &msglen_s );
  // replylen_s set
  reply_s = get_message( gc->cur_task->sp, 64, &replylen_s );

  debug("SENDER: tid: %d, msg: %x, msglen: %d, reply: %x, replylen: %d",
    tid_s, msg_s, msglen_s, reply_s, replylen_s);

  int tid_check = check_tid( gc, tid_s );
  if ( tid_check < 0 ) {
    gc->cur_task->retval = tid_check;
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
  task_descriptor_t * td_r = &(gc->tds[TID_IDX(tid_s)]);
  if( td_r->status == TD_SEND_BLOCKED ) {
    unsigned int *ptid_r;
    int msglen_r;
    char *msg_r;

    /* get receiver's arguments: *tid, *msg and msglen */
    ptid_r = (unsigned int *) get_message_tid( td_r->sp, 56 );
    msg_r = get_message( td_r->sp, 60, &msglen_r );
    //debug("RECEIVER: tid_r: %x, msg_r: %x, msglen_r: %d", ptid_r, msg_r, msglen_r);

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
    //debug( "New Sender on Receive Queue" );
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
    //debug( "receive TD is now blocked" );
    // Anything else?
  } else {
    //debug( "Receiving a send waiting in queue" );
    task_descriptor_t *s_td = r_td->first_sender_in_queue;
    assert(s_td->status == TD_RECEIVE_BLOCKED);
    r_td->first_sender_in_queue = s_td->next_sender;
    s_td->next_sender = NULL;
    if (r_td->first_sender_in_queue == NULL) {
      r_td->last_sender_in_queue = NULL;
    }

    unsigned int tid_s;
    int msglen_s, replylen_s;
    char *msg_s, *reply_s;

    /* get the first two arguments: tid and *msg */
    tid_s = get_message_tid( s_td->sp, 72 );
    msg_s = get_message( s_td->sp, 56, &msglen_s );
    reply_s = get_message( s_td->sp, 64, &replylen_s );

    debug("SENDER: tid: %d, msg: %x, msglen: %d, reply: %x, replylen: %d",
       tid_s, msg_s, msglen_s, reply_s, replylen_s);

    unsigned int *ptid_r;
    int msglen_r;
    char *msg_r;

    /* get receiver's arguments: *tid, *msg and msglen */
    ptid_r = (unsigned int *) get_message_tid( r_td->sp, 56 );
    msg_r = get_message( r_td->sp, 60, &msglen_r );
    //debug("tid: %x, msg: %x, msglen: %d", ptid_r, msg_r, msglen_r);

    *ptid_r = s_td->id;
    int msg_copied = min(msglen_s, msglen_r);
    kmemcpy( /*dst*/ msg_r, /*src*/ msg_s, msg_copied );

    s_td->status = TD_REPLY_BLOCKED;
    r_td->retval = msg_copied;
    add_to_priority( gc, r_td );
  }
}

void handle_reply( global_context_t *gc ) {
  unsigned int target_tid;
  int replylen_r, replylen_s;
  char *reply_r, *reply_s;

  /* get replyer's arg: target tid */
  target_tid = get_message_tid( gc->cur_task->sp, 56 );
  
  int tid_check = check_tid( gc, target_tid );
  if ( tid_check < 0 ) {
    gc->cur_task->retval = tid_check;
    add_to_priority( gc, gc->cur_task );
    return;
  }

  /* now we know the sender exists 
   * 1. if the task is not REPLY_BLOCKED, return -3
   * 2. if sender's buffer_len < replylen, return -4
   * 3. else, cpy reply msg to sender's space, return 0
   */
  task_descriptor_t * target_td = &(gc->tds[TID_IDX(target_tid)]);
  if( target_td->status != TD_REPLY_BLOCKED ) {
    gc->cur_task->retval = -3;
    add_to_priority( gc, gc->cur_task );
    return;
  }

  /* get the next two replyer's arguments: *reply, and replylen*/
  reply_r = get_message( gc->cur_task->sp, 60, &replylen_r );

  /* get the sender's args: *reply_s, replylen_s */
  reply_s = get_message( target_td->sp, 64, &replylen_s );

  //debug("reply_r: %x, replylen_r: %d\n\r reply_s: %x, replylen_s: %d",
  //   reply_r, replylen_r, reply_s, replylen_s);

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

// TODO: Get the interrupt type and switch on that
void handle_hwi( global_context_t *gc ) {
  int *timer_base = (int *)(TIMER3_BASE + CLR_OFFSET);
  *timer_base = 1;
  add_to_priority( gc, gc->cur_task );
}

void handle_create( global_context_t *gc ) {
#ifndef CLANG
  register unsigned int priority_reg asm("r0");
  unsigned int priority;
  register void (*code_reg)() asm("r1");
  void (*code)();
  register unsigned int *cur_sp_reg asm("r3") = (gc->cur_task)->sp;

  asm volatile(
    "msr cpsr_c, #0xdf\n\t"
    "add r3, %2, #56\n\t" // 14 registers + pc + retval saved
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
#endif
}

void handle_pass( global_context_t *gc ) {
  add_to_priority( gc, gc->cur_task );
}

void handle_exit( global_context_t *gc ) {
  // Not much to do, make sure to remove from ALL queues
  // including message queues

  // NOTE: This task is already removed from priority queue.
  // Since exit does NOT reclaim memory, we don't need to add it back
  // to the free list

  // Any senders blocked for this receiver should get error message.
  task_descriptor_t *send_td = gc->cur_task->first_sender_in_queue;
  while( send_td != NULL ) {
    send_td->retval = -3;
    add_to_priority( gc, send_td );
    send_td = send_td->next_sender;
  }
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
