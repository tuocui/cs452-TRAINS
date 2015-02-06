#include "tools.h"
//TODO, modularize kernel service such as nameserver and syscall
#include "nameserver.h"
#include "syscall.h"
#include "timer.h"
#include "clock_server.h"


void clock_clients_init( clock_client_t *clients) {
  int i = 0;
  for( ; i < TD_MAX; ++i ) {
    clients[i].c_tid = 0;
    clients[i].next_client_q = NULL;
  }
}

void notifier_init( ) {
  int clock_server_tid = MyParentTid( );
  //TODO: this should be a level 1 assert
  assert( clock_server_tid == WhoIs( (char *)CLOCK_SERVER ));
  
  clock_msg_t msg;
  int msg_size = sizeof(msg);
  msg.request_type = CM_UPDATE;

  char rpl = (char)NOTIFIER_MAGIC;

  FOREVER {
    msg.value = AwaitEvent( TIMER_INTERRUPT );

    //TODO: level 1 assert
    assert( msg.value > 0, "ERROR: interrupt eventid is incorrect" );

    Send( clock_server_tid, (char*)&msg, msg_size, &rpl, 1 );
    
    //TODO: should be level 1 assert 
    assert( rpl == (char)NOTIFIER_MAGIC );
  }
}


void insert_client( clock_client_t *new_client, clock_client_t *first_client_q ) {
  if( !first_client_q )
    first_client_q = new_client;

  else if( new_client->future_ticks <= first_client_q->future_ticks ) {
    new_client->next_client_q = first_client_q;
    first_client_q = new_client;
  }

  else {
    clock_client_t * cur_client = first_client_q;

    while( cur_client->next_client_q != NULL &&
        new_client->future_ticks > cur_client->next_client_q->future_ticks ) {
      cur_client = cur_client->next_client_q;
    }

    new_client->next_client_q = cur_client->next_client_q;
    cur_client->next_client_q = new_client;
  }
 
}

void clock_server_init( ) {
  if( RegisterAs( (char *)CLOCK_SERVER ) == -1) {
    bwputstr( COM2, "ERROR: failed to register clock_server, aborting." );
    Exit( );
  }

  unsigned int clock_ticks = 0;

  clock_client_t clients[TD_MAX];
  clock_clients_init( clients );
  clock_client_t *first_client_q = NULL;

  int notifier_tid = Create( 1, &notifier_init );
  int client_tid, msg_size;
  clock_msg_t receive_msg, reply_msg;
  reply_msg.request_type = CM_REPLY;
  msg_size = sizeof(receive_msg);
  char notifier_rpl = (char)NOTIFIER_MAGIC;


  FOREVER {
    Receive( &client_tid, (char *)&receive_msg, msg_size );

    switch( receive_msg.request_type ) {

    case CM_UPDATE:
      /* reply to notifier */
      Reply( notifier_tid, &notifier_rpl, 1 );

      /* update the clock */
      clock_ticks += receive_msg.value;

      /* reply to clients */
      //TODO: make below into a separate function
      while( first_client_q && first_client_q->future_ticks <= clock_ticks ) {
        reply_msg.value = clock_ticks;
        
        Reply( first_client_q->c_tid, (char *)&reply_msg, msg_size );

        first_client_q = first_client_q->next_client_q;
      }
      break;

    case CM_TIME:
      reply_msg.value = clock_ticks;
      Reply( client_tid, (char *)&reply_msg, msg_size );
      break;

    case CM_DELAY:
      {
        clock_client_t *new_client = &(clients[TID_IDX(client_tid)]);
        new_client->c_tid = TID_IDX(client_tid);
        new_client->future_ticks = clock_ticks + receive_msg.value;
        
        insert_client( new_client, first_client_q );
        break;
      }

    case CM_DELAY_UNTIL:
      {
        clock_client_t *new_client = &(clients[TID_IDX(client_tid)]);
        new_client->c_tid = TID_IDX(client_tid);
        new_client->future_ticks = receive_msg.value;


        
        insert_client( new_client, first_client_q );
        break;
      }
    default:
      bwprintf( COM2, "ERROR: clock_server receives invalid msg type");
    break;
    }
  }
}

