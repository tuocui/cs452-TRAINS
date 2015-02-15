#include <tools.h>
#include <user_task.h>
#include <syscall.h>
#include <nameserver.h>
#include <rps.h>
#include "clock_server.h"

#define CYCLES 1000

//#define A1 1
//#define A2 1 
#define A3 1

//TODO remove testing struct
struct Server {
  char arr[4];
};

void idle_task( ) {
  debug( "IN IDLE TASK" );
  FOREVER {
  }
  Exit( );
}

void gen_user_task( ){
  struct Server server_r;
  int i = 0;
  int tid;
  char * msg;
  int msglen = sizeof(server_r);
  msg = (char*)&server_r;
  for( ; i < 5; ++i ){
    debug( "In gen_user_task, iter: %d", i );
    Pass( );
  }
  debug("tid: %x, msg: %x, msglen: %d", &tid, &server_r, msglen);
  int rtn = Receive( &tid, (char *) &server_r, msglen );
  debug("rtn: %d", rtn);
  //debug("server_r.a: %c, server_r.b: %c", server_r.a, server_r.b);
  Exit( );
}

void user_receive_task( ){
  int sender_tid;
  struct Server server_r;
  struct Server server_reply;
  int cycles = CYCLES;
  int rtn;

  //debug( "tid: %x, msg: %x, msglen: %d", &sender_tid, &server_r,  sizeof(server_r));

  int msglen = sizeof(server_r);

  while(--cycles >= 0){
    rtn = Receive( &sender_tid, (char *) &server_r, msglen );
    debug("rtn: %d", rtn);
    //debug("server_r.a: %d, server_r.b: %d", server_r.a, server_r.b);
    rtn = Reply( sender_tid, (char*)&server_reply, sizeof(server_reply) );
    debug("Reply's rtn: %d", rtn );
  }
  //rtn = Receive( &sender_tid, (char *) &server_r, msglen );
  //debug("rtn: %d", rtn);
  //debug("server_r.a: %d, server_r.b: %d", server_r.a, server_r.b);
  //char c = bwgetc( COM2 );
  //bwputc( COM2, c );
  Exit( );
}


void user_send_task( ){
  struct Server server_s;
  struct Server server_reply;
  //int my_tid = MyTid( );
  //int receiver_tid = 34;
  debug( "In Send" );
  int receiver_tid = Create( 1, &user_receive_task );
  int cycles;
  int rtn;
  cycles = CYCLES;
  //server_s.a = (char) my_tid;
  //server_s.b = (char) my_tid;
  //debug("tid: %d, msg: %x, msglen: %d, reply: %x, replylen: %d", receiver_tid, &server_s,  sizeof(server_s), &server_reply,  sizeof(server_reply) );
  

  while(--cycles >= 0){
    rtn = Send( receiver_tid, (char *) &server_s,  sizeof(server_s), (char *) &server_reply, sizeof(server_reply));
    debug( "Send's rtn: %d", rtn );
  }

  Exit( );

}

#ifdef A1
void a1_user_task( ){
  unsigned int my_tid;
  unsigned int parent_tid;
  my_tid = MyTid( );
  parent_tid = MyParentTid( );
  bwprintf( COM2, "My TID: %d, Parent TID: %d\n\r", my_tid, parent_tid );
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(my_tid), TID_GEN(my_tid));
  Pass( );
  bwprintf( COM2, "My TID: %d, Parent TID: %d\n\r", my_tid, parent_tid );
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(my_tid), TID_GEN(my_tid));
  Exit( );
}
#endif /* A1 */
#ifdef A2
void a2_test_task( ) {
  start_clock( TIMER_LOAD_VAL );

  int  sender_tid1;
  unsigned int start_tick, end_tick; 
  
  start_tick = (unsigned int)get_timer_val( );
  sender_tid1 = Create( 3, &user_send_task );
  end_tick = (unsigned int)get_timer_val( );

  bwprintf( COM2, "average duration in ticks:: %d\n\r", (start_tick - end_tick) );
  //TODO: timer ends here, to make it more accurate, put it inside kernel
  //int sender_tid2 = Create( 1, &user_send_task );
  //debug( "Receiver tid: %d, sender1 tid: %d, sender2 tid: %d", receiver_tid, sender_tid1, sender_tid2 );
  Exit( );
}

void a2_user_task( ) {
  bwsetfifo( COM2, OFF );
  // Create nameserver
  int nameserver_tid = Create( 1, &nameserver_main );
  debug( "Nameserver tid: %d", nameserver_tid );
  int rps_server_tid = Create( 2, &rps_server );
  debug( "RPS Server tid: %d", rps_server_tid );
  int rps_client1_tid = Create( 10, &rps_client1 );
  debug( "RPS client1 tid: %d", rps_client1_tid );
  int rps_client2_tid = Create( 10, &rps_client1 );
  debug( "RPS client2 tid: %d", rps_client2_tid );
  int rps_client3_tid = Create( 10, &rps_client1 );
  debug( "RPS client3 tid: %d", rps_client3_tid );
  int rps_client4_tid = Create( 10, &rps_client1 );
  debug( "RPS client4 tid: %d", rps_client4_tid );
  int rps_client5_tid = Create( 10, &rps_client1 );
  debug( "RPS client5 tid: %d", rps_client5_tid );
  // Create RPS server
  // Create RPS clients
  Exit( );
}
#endif /* A2 */

#ifdef A3

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
  int total_ticks;
  for( ; i < num_delays; ++i ) {
    errno = Delay( delay_time );
    total_ticks = Time( );
    bwprintf( COM2, "Total_time: %d\tTid: %d, with delay time %d, delayed %d/%d times\r\n",
        total_ticks, my_tid, delay_time, i+1, num_delays);
  }
  Exit( );
}

void a3_test_task( ) {
  start_clock( 5080 );
  debug( "hwi test" );
  unsigned int i = 0, j = 0, ae_rtn;
  while( i < 3000 && j < 3000 ) {
    ae_rtn = AwaitEvent( TIMER3_INT_IND );
    ++i;
    ++j;
    debug( "j: %d, i: %d, rtn: %d", j, i, ae_rtn );
  }
  debug( "After loop: j: %d, i: %d", j, i );
}

void a3_user_task( ) {
  //bwsetfifo( COM2, OFF );
  //
  // Create nameserver
  int nameserver_tid = Create( 2, &nameserver_main );
  debug( "Nameserver tid: %d", nameserver_tid );

  // Create idle
  int idle_id = Create( PRIORITY_MAX, &idle_task );
  debug( "Idle tid: %d", idle_id );

  // Create clock server
  int clock_server_tid = Create( 2, &clock_server );
  debug( "Clock Server tid: %d", clock_server_tid );

  // Create Clients
  int client_tid;
  int msg;
  int msg_len = sizeof(msg);
  clock_client_msg_t rpl;
  int rpl_len = sizeof(rpl);

#define CLOCK_CLIENT_WAIT( _priority, _delay_time, _num_delays) {         \
  int clock_client_tid##_priority = Create( _priority, &clock_client );   \
  debug( "Clock Client##priority tid: %d", clock_client_tid##_priority ); \
  Receive( &client_tid, (char *)&msg, msg_len );                          \
  debug( "Received from: %d, %d, %d", msg, client_tid,                    \
      clock_client_tid##_priority );                                      \
  if( client_tid == msg && client_tid == clock_client_tid##_priority ) {  \
    rpl.delay_time = _delay_time;                                         \
    rpl.num_delays = _num_delays;                                         \
    Reply( client_tid, (char *)&rpl, rpl_len );                           \
  }                                                                       \
}

  CLOCK_CLIENT_WAIT( 3, 10, 20 );
  CLOCK_CLIENT_WAIT( 4, 23, 9 );
  CLOCK_CLIENT_WAIT( 5, 33, 6 );
  CLOCK_CLIENT_WAIT( 6, 71, 3 );

}

#endif /* A3 */

void first_user_task( ){
  bwsetfifo( COM2, OFF );
#ifdef A1
  int first_tid = MyTid( );
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(first_tid), TID_GEN(first_tid));

  int created_tid;
  created_tid = Create( 10, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Lower than First\n\r", created_tid);
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(created_tid), TID_GEN(created_tid));

  created_tid = Create( 10, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Lower than First\n\r", created_tid);
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(created_tid), TID_GEN(created_tid));

  created_tid = Create( 1, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Higher than First\n\r", created_tid);
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(created_tid), TID_GEN(created_tid));

  created_tid = Create( 1, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Higher than First\n\r", created_tid);
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(created_tid), TID_GEN(created_tid));
#endif /* A1 */

#ifdef A2
  debug( "First User Task" );
  //a2_test_task( );
  //a2_user_task( );

#endif /* A2 */

#ifdef A3

  a3_user_task( );
  //int test_id;
  //test_id = Create( 6, &a3_test_task );
  //bwprintf( COM2, "idle_id: %d, test_id: %d\r\n", idle_id, test_id );
#endif /* A3 */

  bwprintf( COM2, "Exit first_user_task\n\r");
  Exit( );
}
