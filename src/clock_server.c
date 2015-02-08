#include "tools.h"
//TODO, modularize kernel service such as nameserver and syscall
#include "nameserver.h"
#include "syscall.h"
#include "clock_server.h"
#include "ts7200.h"
#include "timer.h"

void clock_clients_init( clock_client_t *clients) {
  int i = 0;
  for( ; i < TD_MAX; ++i ) {
    clients[i].c_tid = 0;
    clients[i].next_client_q = NULL;
  }
}

void notifier( ) {
  int clock_server_tid = MyParentTid( );
  //TODO: this should be a level 1 assert
  
  clock_msg_t msg;
  int msg_size = sizeof(msg);
  msg.request_type = CM_UPDATE;

  char rpl = (char)NOTIFIER_MAGIC;
  FOREVER {
    msg.value = AwaitEvent( TIMER3_INT_IND );

    assert(0, msg.value > 0, "ERROR: interrupt eventid is incorrect" );

    Send( clock_server_tid, (char*)&msg, msg_size, &rpl, 1 );
    
    // assert(1, rpl == (char)NOTIFIER_MAGIC );
  }
}


void insert_client( clock_client_t **new_client, clock_client_t **first_client_q ) {
  if( *first_client_q == NULL ) {
    *first_client_q = *new_client;
    //debug(" inserted client: first_user_client id: %d, future_ticks", (*first_client_q)->c_tid, (*first_client_q)->future_ticks );
  } else if( (*new_client)->future_ticks <= (*first_client_q)->future_ticks ) {
    (*new_client)->next_client_q = (*first_client_q);
    (*first_client_q) = (*new_client);
  } else {
    clock_client_t * cur_client = (*first_client_q);
    while( cur_client->next_client_q != NULL &&
        (*new_client)->future_ticks > cur_client->next_client_q->future_ticks ) {
      cur_client = cur_client->next_client_q;
    }

    (*new_client)->next_client_q = cur_client->next_client_q;
    cur_client->next_client_q = (*new_client);
  }
 
}

void clock_server( ) {
  if( RegisterAs( (char *)CLOCK_SERVER ) == -1) {
    bwputstr( COM2, "ERROR: failed to register clock_server, aborting." );
    Exit( );
  }

  unsigned int clock_ticks = 0;

  clock_client_t clients[TD_MAX];
  clock_clients_init( clients );
  clock_client_t *first_client_q = NULL;
  int client_tid, msg_size;
  char notifier_rpl = (char)NOTIFIER_MAGIC;
  clock_msg_t receive_msg, reply_msg;
  reply_msg.request_type = CM_REPLY;
  msg_size = sizeof(receive_msg);

  int notifier_tid = Create( 1, &notifier );
  debug( "notifier_tid: %d, server_tid: %d", notifier_tid, MyTid( ) );
  start_clock( TICKS_TEN_MS );
  FOREVER {
    Receive( &client_tid, (char *)&receive_msg, msg_size );

    switch( receive_msg.request_type ) {
    case CM_UPDATE:
      /* reply to notifier */
      Reply( notifier_tid, &notifier_rpl, 1 );

      /* update the clock */
      clock_ticks += receive_msg.value;
      //debug( "clock_ticks: %d", clock_ticks );

      /* reply to clients */
      //TODO: make below into a separate function
      //debug("first_user_client id: %d, future_ticks: %d", first_client_q->c_tid, first_client_q->future_ticks );
      while( first_client_q != NULL && first_client_q->future_ticks <= clock_ticks ) {
        //debug( "in while loop" );
        reply_msg.value = clock_ticks;
        Reply( first_client_q->c_tid, (char *)&reply_msg, msg_size );
        first_client_q->c_tid = 0;
        first_client_q->future_ticks = 0;
        clock_client_t *old_client = first_client_q;
        first_client_q = first_client_q->next_client_q;
        old_client->next_client_q = NULL;
      }
      break;

    case CM_TIME:
      reply_msg.value = clock_ticks;
      Reply( client_tid, (char *)&reply_msg, msg_size );
      break;

    case CM_DELAY:
      {
        clock_client_t *new_client = &(clients[TID_IDX(client_tid)]);
        new_client->c_tid = client_tid;
        new_client->future_ticks = clock_ticks + receive_msg.value;
        
        insert_client(&new_client, &first_client_q);
        break;
      }
    case CM_DELAY_UNTIL:
      {
        if( receive_msg.value <= clock_ticks ) {
          reply_msg.value = clock_ticks;
          Reply( client_tid, (char *)&reply_msg, msg_size );
          break;
        }
        clock_client_t *new_client = &(clients[TID_IDX(client_tid)]);
        new_client->c_tid = TID_IDX(client_tid);
        new_client->future_ticks = receive_msg.value;
        insert_client( &new_client, &first_client_q );
        break;
      }
    default:
      bwprintf( COM2, "ERROR: clock_server receives invalid msg type");
      reply_msg.value = -1;
      Reply( client_tid, (char *)&reply_msg, msg_size );
      break;
    }
  }
}

int Delay( int ticks ) {
  if( ticks <= 0 ) {
    return 0;
  }
  int clock_server_id = WhoIs( (char *)CLOCK_SERVER );
  if( clock_server_id < 0 ) {
    return -1;
  }
  clock_msg_t msg;
  clock_msg_t rpl;
  int msg_len = sizeof( msg );
  msg.request_type = CM_DELAY;
  msg.value = ticks;
  //debug( "Sending delay: msg.value: %d", msg.value );
  Send( clock_server_id, (char *)&msg, msg_len, (char *)&rpl, msg_len );
  return 0;
}

int DelayUntil( int ticks ) {
  if( ticks <= 0 ) {
    return 0;
  }
  int clock_server_id = WhoIs( (char *)CLOCK_SERVER );
  if( clock_server_id < 0 ) {
    return -1;
  }
  clock_msg_t msg;
  clock_msg_t rpl;
  int msg_len = sizeof( msg );
  msg.request_type = CM_DELAY_UNTIL;
  msg.value = ticks;
  Send( clock_server_id, (char *)&msg, msg_len, (char *)&rpl, msg_len );
  return 0;
}

int Time( ) {
  int clock_server_id = WhoIs( (char *)CLOCK_SERVER );
  if( clock_server_id < 0 ) {
    return -1;
  }
  clock_msg_t msg;
  clock_msg_t rpl;
  int msg_len = sizeof( msg );
  msg.request_type = CM_TIME;
  Send( clock_server_id, (char *)&msg, msg_len, (char *)&rpl, msg_len );
  return rpl.value;
}

void clock_client( ) {
  int my_tid = MyTid();
  int msg = my_tid;
  int msg_len = sizeof(msg);
  clock_client_msg_t rpl;
  int rpl_len = sizeof(rpl);
  int parent_id = MyParentTid( );
  Send( parent_id, (char *)&msg, msg_len, (char *)&rpl, rpl_len );
  int delay_time = rpl.delay_time;
  int num_delays = rpl.num_delays;
  debug( "got back from first user task: %d, %d", delay_time, num_delays );
  int i = 0;
  int errno;
  for( ; i < num_delays; ++i ) {
    errno = Delay( delay_time );
    bwprintf( COM2, "Task: %d, with delay time %d, delayed %d times!\r\n", my_tid, delay_time, i );
  }
  Exit( );
}


