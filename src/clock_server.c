#include "tools.h"
//TODO, modularize kernel service such as nameserver and syscall
#include "nameserver.h"
#include "syscall.h"
#include "timer.h"
#include "clock_server.h"


void clock_clients_init( clock_client_t *clients) {
  int i = 0;
  for( ; i < TD_MAX - 1; ++i ) {
    clients[i].c_tid = 0;
    clients[i].ticks = 0;
    clients[i].next_client_q = NULL;
    clients[i].next_client_free = &( clients[i + 1] );
    clients[i].request_type = CC_INACTIVE;
  }
  clients[i].c_tid = 0;
  clients[i].next_client_q = NULL;
  clients[i].next_client_free = NULL;
}

void notifier_init( ) {
  int clock_server_tid = MyParentTid( );
  //TODO: this should be a level 1 assert
  assert( CLOCK_SERVER_tid == Whois( (char *)CLOCK_SERVER ));
  
  FOREVER {
    int event_retval = AwaitEvent( TIMER_INTERRUPT );

    assert( event_retval >= 0 );

    char msg, rpl;
    msg = (char)UPDATE_TIME;
    Send( clock_server_tid, &msg, 1, &rpl, 1 );
    
    //TODO: should be level 1 assert 
    assert( rpl == (char)NOTIFIER_MAGIC );
  }
}

void clock_server_init( ) {
  if( RegisterAs( (char *)CLOCK_SERVER ) == -1) {
    bwputstr( COM2, "ERROR: failed to register clock_server, aborting." );
    Exit( );
  }

  clock_client_t clients[TD_MAX];
  initialize_clients( clients );
  clock_client_t *first_client_q = NULL;
  clock_client_t *last_client_q = NULL;
  clock_client_t *first_client_free = &( clients[0] );
  clock_client_t *last_client_free = &( clients[TD_MAX - 1] );

  int notfier_tid = Create( 1, &notifier_init );
  int client_tid;

  FOREVER {
    Receive( (int *)&client_tid, 
  }
}




