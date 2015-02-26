#include <tools.h>
#include <kernel.h>
#include <ts7200.h>

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
  assert(0, tid_idx >= 0 );
  assert(0, tid_idx <= ( TD_MAX - 1 ));

  // If generation == 0, then impossible
  if( TID_GEN(tid) == 0 ){
    return -1;
  }

  /* if task has a different gen, or it's a zombie, return -2, schedule sender */
  task_descriptor_t * td = &(gc->tds[TID_IDX(tid)]);
  if( TID_GEN(td->id) != TID_GEN(tid) ||
    td->status == TD_ZOMBIE ) {
    return -2;
  }

  //TODO: think about -3 return val

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
    assert(0, s_td->status == TD_RECEIVE_BLOCKED);
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

    //debug("SENDER: tid: %d, msg: %x, msglen: %d, reply: %x, replylen: %d",
    //   tid_s, msg_s, msglen_s, reply_s, replylen_s);

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
      ++(gc->num_tasks);
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

  --(gc->num_tasks);
  /* Only idle task not exited */
  if( gc->num_tasks <= 1 ) {
    ((gc->priorities)[PRIORITY_MAX]).first_in_queue = NULL;
    ((gc->priorities)[PRIORITY_MAX]).last_in_queue = NULL;
  }
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

void handle_await_event( global_context_t *gc ) {
  //debug( "In handle await event" );
  register int event_type_reg asm("r0");
  int event_type;
  register unsigned int *cur_sp_reg asm("r3") = (gc->cur_task)->sp;

  asm volatile(
    "msr cpsr_c, #0xdf\n\t"
    "add r3, %1, #56\n\t" // 14 registers + pc + retval saved
    "ldmfd r3, {%0}\n\t"
    "msr cpsr_c, #0xd3\n\t"
    : "+r"(event_type_reg), "+r"(cur_sp_reg)
  );
  event_type = event_type_reg;
  //debug( "Obtained event_type: %d", event_type );
  switch( event_type ) {
    case COM1_OUT_IND:
      // Turn on COM1 Output interrupts
      // debug("Handling awaitevent");
	    *( (unsigned int*)(UART1_BASE + UART_CTLR_OFFSET)) |= TIEN_MASK | MSIEN_MASK;
      //debug("UART1_CTLR_OFFSET: %x", *( (unsigned int*)(UART1_BASE + UART_CTLR_OFFSET)) );
      break;
    case COM1_IN_IND:
      debug("com1 in task awaiting");
      *( (unsigned int*)(UART1_BASE + UART_CTLR_OFFSET)) |= RIEN_MASK;
      break;
    case COM2_OUT_IND:
      *( (unsigned int*)(UART2_BASE + UART_CTLR_OFFSET)) |= TIEN_MASK;
      break;
    case COM2_IN_IND:
      *( (unsigned int*)(UART2_BASE + UART_CTLR_OFFSET)) |= RIEN_MASK | RTIEN_MASK;;
      break;

    default:
      break;
  }
  (gc->interrupts)[event_type] = gc->cur_task;
  (gc->cur_task)->status = TD_EVENT_BLOCKED;
}

void handle_timer_int( global_context_t *gc ) {
  /* turn off interrupt on TC3 */
  int *timer_clr = (int *)(TIMER3_BASE + CLR_OFFSET);
  task_descriptor_t *td = gc->interrupts[TIMER3_INT_IND];
  if( td != NULL ) {
    td->retval = gc->num_missed_clock_cycles + 1;
    (gc->interrupts)[TIMER3_INT_IND] = NULL;
    gc->num_missed_clock_cycles = 0;
    add_to_priority( gc, td );
    assert(0, td != gc->cur_task );
  } else {
    ++(gc->num_missed_clock_cycles);
  }
  ++(gc->num_ticks);
  if( gc->cur_task->priority == PRIORITY_MAX ) {
    ++(gc->num_ticks_idle);
  }
  *timer_clr = 1;
}

void handle_uart1_combined_int( global_context_t *gc ) {
  int uart1_int_status = *((unsigned int *)( UART1_BASE + UART_INTR_OFFSET ));
  int *uart1_status_flags = (int *)(UART1_BASE + UART_FLAG_OFFSET);
  //int *uart1_modem_status_flags = (int *)(UART1_BASE + UART_MDMSTS_OFFSET);
  //debug("uart1 int status: %x", uart1_int_status );

  if( uart1_int_status & UART_TIS_MASK && !(*uart1_status_flags & TXFF_MASK) ) {
    //debug("Transmit interrupt hit");
    gc->com1_status |= COM1_TRANSMIT_MASK;
    *( (unsigned int*)(UART1_BASE + UART_CTLR_OFFSET)) &= ~TIEN_MASK;
  }

  //if( uart1_int_status & UART_MIS_MASK && *uart1_modem_status_flags & DCTS_MASK && *uart1_status_flags & CTS_MASK ) {
  if( uart1_int_status & UART_MIS_MASK && *uart1_status_flags & CTS_MASK ) {
    //debug("MSIEN interrupt hit");
    gc->com1_status |= COM1_CTS_MASK;
    // Kill interrupt
    *( (unsigned int*)(UART1_BASE + UART_CTLR_OFFSET)) &= ~MSIEN_MASK;
  }

  // Don't need Receive timout, since COM1 doesn't have FIFO turned on yet.
  if( uart1_int_status & UART_RIS_MASK && *uart1_status_flags & RXFF_MASK) {
    debug("RIEN interrupt hit");
    char c = *((unsigned int *)( UART1_BASE + UART_DATA_OFFSET ));
    task_descriptor_t *td = gc->interrupts[COM1_IN_IND];
    gc->interrupts[COM1_IN_IND] = NULL;
    assert( 0, td != NULL, "FUCK IN TD IS NULL" );
    //gc->com1_status |= COM1_RECEIVE_MASK;
    *( (unsigned int*)(UART1_BASE + UART_CTLR_OFFSET)) &= ~RIEN_MASK;
    *((unsigned int *)( UART1_BASE + UART_INTR_OFFSET )) &= ~UART_RIS_MASK;
    td->retval = c;
    add_to_priority( gc, td );
  }

  if( gc->com1_status & COM1_TRANSMIT_MASK && gc->com1_status & COM1_CTS_MASK ) {
    task_descriptor_t *td = gc->interrupts[COM1_OUT_IND];
    assert( 0, td != NULL, "FUCK OUT TD IS NULL" );
    gc->com1_status &= (~COM1_CTS_MASK & ~COM1_TRANSMIT_MASK);
    gc->interrupts[COM1_OUT_IND] = NULL;
    td->retval = 0;
    *( (unsigned int*)(UART1_BASE + UART_CTLR_OFFSET)) &= ~TIEN_MASK;
    *( (unsigned int*)(UART1_BASE + UART_CTLR_OFFSET)) &= ~MSIEN_MASK;
    add_to_priority( gc, td );
  }
}

void handle_uart2_combined_int( global_context_t *gc ) {
  int uart2_int_status = *((unsigned int *)( UART2_BASE + UART_INTR_OFFSET ));
  int uart2_status_flags = *((unsigned int *)(UART2_BASE + UART_FLAG_OFFSET));
  debug("uart2 int status: %x", uart2_int_status );

  if( uart2_int_status & UART_TIS_MASK && !(uart2_status_flags & TXFF_MASK) ) {
    debug("Transmit interrupt hit");
    task_descriptor_t *td = gc->interrupts[COM2_OUT_IND];

    assert( 0, td != NULL, "FUCK OUT TD IS NULL" );

    gc->interrupts[COM2_OUT_IND] = NULL;
    td->retval = 0;
    *( (unsigned int*)(UART2_BASE + UART_CTLR_OFFSET)) &= ~TIEN_MASK;
    add_to_priority( gc, td );
  }

  if( ( uart2_int_status & UART_RIS_MASK || uart2_int_status & UART_RTIS_MASK ) && !(uart2_status_flags & RXFE_MASK) ) {
    debug("RIEN interrupt hit");
    task_descriptor_t *td = gc->interrupts[COM2_IN_IND];
    //debug( "In handle await event" );
    register int event_type_reg asm("r0");
    int event_type;
    register char *event_buf_reg asm("r1");
    char *event_buf;
    register int event_len_reg asm("r2");
    int event_len;
    register unsigned int *cur_sp_reg asm("r3") = td->sp;
    asm volatile(
      "msr cpsr_c, #0xdf\n\t"
      "add r3, %3, #56\n\t" // 14 registers + pc + retval saved
      "ldmfd r3, {%0, %1, %2}\n\t"
      "msr cpsr_c, #0xd3\n\t"
      : "+r"(event_type_reg), "+r"(event_buf_reg), "+r"(event_len_reg), "+r"(cur_sp_reg)
    );
    event_type = event_type_reg;
    event_buf = event_buf_reg;
    event_len = event_len_reg;
    gc->interrupts[COM2_IN_IND] = NULL;
    assert( 0, td != NULL, "FUCK IN TD IS NULL" );
    if( uart2_int_status & UART_RIS_MASK ){
      *((unsigned int*)(UART2_BASE + UART_CTLR_OFFSET)) &= ~RIEN_MASK;
      *((unsigned int *)( UART2_BASE + UART_INTR_OFFSET )) &= ~UART_RIS_MASK;
    } else if ( uart2_int_status & UART_RTIS_MASK ) {
      *((unsigned int*)(UART2_BASE + UART_CTLR_OFFSET)) &= ~RTIEN_MASK;
      *((unsigned int *)( UART2_BASE + UART_INTR_OFFSET )) &= ~UART_RTIS_MASK;
    }
    int i = 0;
    if( td != NULL ) {
      for( ; i < event_len && !(*((unsigned int *)(UART2_BASE + UART_FLAG_OFFSET)) & RXFE_MASK); ++i ) {
        event_buf[i] = *((char *)( UART2_BASE + UART_DATA_OFFSET ));
        //for( j = 0; j < 56; ++j ) ;
      }
      td->retval = i;
      *((unsigned int*)(UART2_BASE + UART_CTLR_OFFSET)) &= ~RIEN_MASK;
      *((unsigned int*)(UART2_BASE + UART_CTLR_OFFSET)) &= ~RTIEN_MASK;
      add_to_priority( gc, td );
    }
  }
}

void handle_hwi( global_context_t *gc, int hwi_type ) {
  switch( hwi_type ){
  case TIMER3_INT:
    handle_timer_int( gc );
    break;
  case UART1_COMBINED_INT:
    handle_uart1_combined_int( gc );
  case UART2_COMBINED_INT:
    handle_uart2_combined_int( gc );
    break;
  default:
    break;
  }
}

void handle_death( global_context_t *gc ) {
  register int magic_death_num_reg asm("r0");
  int magic_death_num;
  register unsigned int *cur_sp_reg asm("r3") = (gc->cur_task)->sp;

  asm volatile(
    "msr cpsr_c, #0xdf\n\t"
    "add r3, %1, #56\n\t" // 14 registers + pc + retval saved
    "ldmfd r3, {%0}\n\t"
    "msr cpsr_c, #0xd3\n\t"
    : "+r"(magic_death_num_reg), "+r"(cur_sp_reg)
  );
  magic_death_num = magic_death_num_reg;
  if( magic_death_num == 0xdeadbeef ) {
    int i;
    for( i = 0; i <= PRIORITY_MAX; ++i ) {
      (gc->priorities)[i].first_in_queue = NULL;
    }
  } else {
    add_to_priority( gc, gc->cur_task );
  }
}

