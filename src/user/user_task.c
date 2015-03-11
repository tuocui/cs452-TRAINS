#include <tools.h>
#include <user_task.h>
#include <syscall.h>
#include <nameserver.h>
#include <rps.h>
#include "clock_server.h"
#include "io.h"
#include "ring_buf.h"
#include "clock.h"
#include "track.h"
#include "screen.h"
#include "util.h"

#include "rail_control.h"
#include "track_node.h"
#include "track_data_new.h"
#include "rail_server.h"
#include "calibration.h"
#include "rail_helper.h"

#define CYCLES 1000

// TODO: Split this into user task and test task files

//#define A1 1
//#define A2 1 
//#define A3 1
//#define A4 1
//#define RING_TEST
#define RAIL_TEST


//TODO remove testing struct
struct Server {
  char arr[4];
};

void idle_task( ) {
  // TODO: Figure out a way so that we can do things in this idle task.
  // PROBLEM: How to make sure that we can sync without blocking.
  // IDEA: Constantly reply to rail server.
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

#ifdef A4

void a4_test_task( ) {
  setspeed( COM1, LOW_SPEED );
  wait_cycles(1000);
  enable_two_stop_bits( COM1 );
  setfifo( COM1, OFF );
  setfifo( COM2, ON );
  int nameserver_tid = Create( 3, &nameserver_main );
  debug( "Nameserver tid: %d", nameserver_tid ); // 33
  int idle_id = Create( PRIORITY_MAX, &idle_task ); // 34
  debug( "Idle tid: %d", idle_id );
  int com2_out_server_tid = Create( 4, &com2_out_server ); // 35
  debug( "COM2_Out Server tid: %d", com2_out_server_tid );
  int com2_in_server_tid = Create( 3, &com2_in_server );  // 37
  debug( "COM2_In Server tid: %d", com2_in_server_tid );
  int com1_out_server_tid = Create( 4, &com1_out_server ); // 39
  debug( "COM1_Out Server tid: %d", com1_out_server_tid );
  int com1_in_server_tid = Create( 3, &com1_in_server );  // 41
  debug( "COM1_In Server tid: %d", com1_in_server_tid );
  int clock_server_tid = Create( 3, &clock_server ); // 43
  debug( "Clock Server tid: %d", clock_server_tid );
  int clock_user_tid = Create( 10, &clock_user_task );
  debug( "Clock user task tid: %d", clock_user_tid );
  Printf( COM2, "\033[2J" );
  initialize_track( );
  int track_sensor_task_tid = Create( 6, &track_sensor_task );
  debug( "Track Sensor task tid: %d", track_sensor_task_tid );
  int parse_user_input_tid = Create( 6, &parse_user_input );
  debug( "User input task tid: %d", parse_user_input_tid );
  int idle_percent_task_tid = Create( PRIORITY_MAX - 1, &idle_percent_task );
  debug( "idle_percent_task tid: %d", idle_percent_task_tid );
  Exit( );
}

void a4_test_task2( ) {
  setspeed( COM1, LOW_SPEED );
  wait_cycles(1000);
  enable_two_stop_bits( COM1 );
  setfifo( COM1, OFF );
  setfifo( COM2, ON );
  int nameserver_tid = Create( 3, &nameserver_main );
  debug( "Nameserver tid: %d", nameserver_tid ); // 33
  int idle_id = Create( PRIORITY_MAX, &idle_task ); // 34
  debug( "Idle tid: %d", idle_id );
  int com2_out_server_tid = Create( 4, &com2_out_server ); // 35
  debug( "COM2_Out Server tid: %d", com2_out_server_tid );
  int com2_in_server_tid = Create( 3, &com2_in_server );  // 37
  debug( "COM2_In Server tid: %d", com2_in_server_tid );
  int com1_out_server_tid = Create( 4, &com1_out_server ); // 39
  debug( "COM1_Out Server tid: %d", com1_out_server_tid );
  int com1_in_server_tid = Create( 3, &com1_in_server );  // 41
  debug( "COM1_In Server tid: %d", com1_in_server_tid );
  int clock_server_tid = Create( 3, &clock_server ); // 43
  debug( "Clock Server tid: %d", clock_server_tid );
  Printf( COM2, "\033[2J" );
  initialize_track( );
  int parse_user_input_tid = Create( 6, &parse_user_input ); // 45
  debug( "User input task tid: %d", parse_user_input_tid );
  int calibrate_train_velocity_tid = Create( 6, &calibrate_train_velocity ); // 46
  debug( "calibrate_train_velocity tid: %d", calibrate_train_velocity_tid );
  int idle_percent_task_tid = Create( PRIORITY_MAX - 1, &idle_percent_task );
  debug( "idle_percent_task tid: %d", idle_percent_task_tid );
  
  Exit( );
}

void a4_test_task3( ) {
  setspeed( COM1, LOW_SPEED );
  wait_cycles(1000);
  enable_two_stop_bits( COM1 );
  setfifo( COM1, OFF );
  setfifo( COM2, ON );
  int nameserver_tid = Create( 3, &nameserver_main );
  debug( "Nameserver tid: %d", nameserver_tid ); // 33
  int idle_id = Create( PRIORITY_MAX, &idle_task ); // 34
  debug( "Idle tid: %d", idle_id );
  int com2_out_server_tid = Create( 4, &com2_out_server ); // 35
  debug( "COM2_Out Server tid: %d", com2_out_server_tid );
  int com2_in_server_tid = Create( 3, &com2_in_server );  // 37
  debug( "COM2_In Server tid: %d", com2_in_server_tid );
  int com1_out_server_tid = Create( 4, &com1_out_server ); // 39
  debug( "COM1_Out Server tid: %d", com1_out_server_tid );
  int com1_in_server_tid = Create( 3, &com1_in_server );  // 41
  debug( "COM1_In Server tid: %d", com1_in_server_tid );
  int clock_server_tid = Create( 3, &clock_server ); // 43
  debug( "Clock Server tid: %d", clock_server_tid );
  Printf( COM2, "\033[2J" );
  initialize_track( );
  int parse_user_input_tid = Create( 6, &parse_user_input ); // 45
  debug( "User input task tid: %d", parse_user_input_tid );
  int calibrate_stopping_distance_tid = Create( 6, &calibrate_stopping_distance ); // 46
  debug( "calibrate_stopping_distance_tid: %d", calibrate_stopping_distance_tid );
  int idle_percent_task_tid = Create( PRIORITY_MAX - 1, &idle_percent_task );
  debug( "idle_percent_task tid: %d", idle_percent_task_tid );
  
  Exit( );
}

void a4_test_task4( ) {
  setspeed( COM1, LOW_SPEED );
  wait_cycles(1000);
  enable_two_stop_bits( COM1 );
  setfifo( COM1, OFF );
  setfifo( COM2, ON );
  int nameserver_tid = Create( 3, &nameserver_main );
  debug( "Nameserver tid: %d", nameserver_tid ); // 33
  int idle_id = Create( PRIORITY_MAX, &idle_task ); // 34
  debug( "Idle tid: %d", idle_id );
  int com2_out_server_tid = Create( 4, &com2_out_server ); // 35
  debug( "COM2_Out Server tid: %d", com2_out_server_tid );
  int com2_in_server_tid = Create( 3, &com2_in_server );  // 37
  debug( "COM2_In Server tid: %d", com2_in_server_tid );
  int com1_out_server_tid = Create( 4, &com1_out_server ); // 39
  debug( "COM1_Out Server tid: %d", com1_out_server_tid );
  int com1_in_server_tid = Create( 3, &com1_in_server );  // 41
  debug( "COM1_In Server tid: %d", com1_in_server_tid );
  int clock_server_tid = Create( 3, &clock_server ); // 43
  debug( "Clock Server tid: %d", clock_server_tid );
  Printf( COM2, "\033[2J" );
  initialize_track( );
  int parse_user_input_tid = Create( 6, &parse_user_input ); // 45
  debug( "User input task tid: %d", parse_user_input_tid );
  int clock_user_tid = Create( 10, &clock_user_task ); // 46
  debug( "Clock user task tid: %d", clock_user_tid );
  int calibrate_accel_time_tid = Create( 6, &calibrate_accel_time ); // 47
  debug( "calibrate_accel_time_tid: %d", calibrate_accel_time_tid );
  int idle_percent_task_tid = Create( PRIORITY_MAX - 1, &idle_percent_task );
  debug( "idle_percent_task tid: %d", idle_percent_task_tid );
  
  Exit( );
}

void a4_test_task5( ) {
  setspeed( COM1, LOW_SPEED );
  wait_cycles(1000);
  enable_two_stop_bits( COM1 );
  setfifo( COM1, OFF );
  setfifo( COM2, ON );
  int nameserver_tid = Create( 3, &nameserver_main );
  debug( "Nameserver tid: %d", nameserver_tid ); // 33
  int idle_id = Create( PRIORITY_MAX, &idle_task ); // 34
  debug( "Idle tid: %d", idle_id );
  int com2_out_server_tid = Create( 4, &com2_out_server ); // 35
  debug( "COM2_Out Server tid: %d", com2_out_server_tid );
  int com2_in_server_tid = Create( 3, &com2_in_server );  // 37
  debug( "COM2_In Server tid: %d", com2_in_server_tid );
  int com1_out_server_tid = Create( 4, &com1_out_server ); // 39
  debug( "COM1_Out Server tid: %d", com1_out_server_tid );
  int com1_in_server_tid = Create( 3, &com1_in_server );  // 41
  debug( "COM1_In Server tid: %d", com1_in_server_tid );
  int clock_server_tid = Create( 3, &clock_server ); // 43
  debug( "Clock Server tid: %d", clock_server_tid );
  Printf( COM2, "\033[2J" );
  initialize_track( );
  int track_sensor_task_tid = Create( 4, &track_sensor_task );
  debug( "Track Sensor task tid: %d", track_sensor_task_tid );
  int rail_server_tid = Create( 3, &rail_server ); // 43
  debug( "Rail Server tid: %d", rail_server_tid );
  int parse_user_input_tid = Create( 6, &parse_user_input ); // 45
  debug( "User input task tid: %d", parse_user_input_tid );
  int clock_user_tid = Create( 10, &clock_user_task ); // 46
  debug( "Clock user task tid: %d", clock_user_tid );
  int idle_percent_task_tid = Create( PRIORITY_MAX - 1, &idle_percent_task );
  debug( "idle_percent_task tid: %d", idle_percent_task_tid );
  
  Exit( );
}
#endif /* A4 */

void ring_buf_test( ) {
  declare_ring_queue(int, test, 2 );
  debug( "buf_count: %d", test_count( ) );
  int idx;
  idx = test_push_back( 1 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );

  idx = test_push_back( 2 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );

  idx = test_push_back( 3 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );
  
  idx = test_push_back( 4 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );

  int elm;
  elm = test_top_front( );
  debug( "buf_count: %d, elm: %d", test_count( ), elm );

  elm = test_top_front( );
  debug( "buf_count: %d, elm: %d", test_count( ), elm );

  elm = test_top_back( );
  debug( "buf_count: %d, elm: %d", test_count( ), elm );

  elm = test_top_back( );
  debug( "buf_count: %d, elm: %d", test_count( ), elm );



  debug(" now what's popping " );
  elm = test_pop_front( );
  debug( "buf_count: %d, elm: %d", test_count( ), elm );

  elm = test_pop_back( );
  debug( "buf_count: %d, elm: %d", test_count( ), elm );

  elm = test_pop_front( );
  debug( "buf_count: %d, elm: %d", test_count( ), elm );

  idx = test_push_back( 12 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );

  idx = test_push_back( 13 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );
  
  idx = test_push_back( 14 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );

  idx = test_push_back( 15 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );

  declare_ring_queue( char, com2_buf, 3 );
  debug( "buf_count: %d", com2_buf_count( ) );
  idx = com2_buf_push_back( 'a' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  idx = com2_buf_push_back( 'b' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );
  
  idx = com2_buf_push_back( 'c' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  idx = com2_buf_push_back( 'd' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  char c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  idx = com2_buf_push_back( 'e' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );
  
  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  idx = com2_buf_push_back( 'f' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  idx = com2_buf_push_back( 'g' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  idx = com2_buf_push_back( 'h' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  idx = com2_buf_push_back( 'i' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );
  
  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  idx = com2_buf_push_back( 'j' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  typedef struct test_struct {
    int a;
  } test_struct_t ;

  test_struct_t st1; st1.a = 1;
  test_struct_t st2; st2.a = 2;
  test_struct_t st3; st3.a = 3;

  declare_ring_queue( test_struct_t *, st_buf, 3 );
  idx = st_buf_push_back( &st1 );
  debug( "buf_count: %d, idx: %d", st_buf_count( ), idx );

  idx = st_buf_push_back( &st2 );
  debug( "buf_count: %d, idx: %d", st_buf_count( ), idx );

  idx = st_buf_push_back( &st3 );
  debug( "buf_count: %d, idx: %d", st_buf_count( ), idx );

  test_struct_t * st0;
  st0 = st_buf_pop_back( );
  debug( "buf_count: %d, st.a: %d", st_buf_count( ), st0->a );

  st0 = st_buf_pop_back( );
  debug( "buf_count: %d, st.a: %d", st_buf_count( ), st0->a );

  st0 = st_buf_pop_back( );
  debug( "buf_count: %d, st.a: %d", st_buf_count( ), st0->a );

  
  debug( "end of test" );
  return;

}

void dijkstra_test( ) {
  debug( "dijkstra_test" );
  setspeed( COM1, LOW_SPEED );
  wait_cycles(1000);
  enable_two_stop_bits( COM1 );
  setfifo( COM1, OFF );
  setfifo( COM2, ON );
  int nameserver_tid = Create( 3, &nameserver_main );
  debug( "Nameserver tid: %d", nameserver_tid ); // 33
  int idle_id = Create( PRIORITY_MAX, &idle_task ); // 34
  debug( "Idle tid: %d", idle_id );
  int com2_out_server_tid = Create( 4, &com2_out_server ); // 35
  debug( "COM2_Out Server tid: %d", com2_out_server_tid );
  int com2_in_server_tid = Create( 3, &com2_in_server );  // 37
  debug( "COM2_In Server tid: %d", com2_in_server_tid );
  int com1_out_server_tid = Create( 4, &com1_out_server ); // 39
  debug( "COM1_Out Server tid: %d", com1_out_server_tid );
  int com1_in_server_tid = Create( 3, &com1_in_server );  // 41
  debug( "COM1_In Server tid: %d", com1_in_server_tid );
  int clock_server_tid = Create( 3, &clock_server ); // 43
  debug( "Clock Server tid: %d", clock_server_tid );

  /* initialize the heap */

  //min_heap_t min_heap;
  //int node_id2idx[NODE_MAX];
  //min_heap_node_t nodes[NODE_MAX];
  //init_min_heap( &min_heap, 0, node_id2idx, nodes );

  ///* test heap_empty */
  //assert(2, heap_empty( &min_heap ));
  //min_heap.size = 1;
  //assert( 2, !heap_empty( &min_heap ));
  //min_heap.nodes[0].dist = 1;
  //assert( 2, extract_min( &min_heap )->dist == 1 );
  //assert( 2, heap_empty( &min_heap ));

  ///* test heapify */
  //min_heap.size = 7;
  //int i = 0;
  //for( ; i < min_heap.size; ++i ){
  //  init_node( &(min_heap.nodes[i]), i, 6-i );
  //}
  //min_heap.nodes[0].dist = 0;
  //make_min_heap( &min_heap, 0 );
  //assert( 2, min_heap.nodes[0].id == 0 );

  //min_heap.nodes[0].dist = 10;
  //make_min_heap( &min_heap, 0 );
  //assert( 2, min_heap.nodes[6].id == 0 );
  //assert( 2, min_heap.nodes[6].dist == 10 );
  //assert( 2, min_heap.nodes[0].id == 2 );
  //assert( 2, min_heap.nodes[0].dist == 4 );
  //assert( 2, min_heap.nodes[2].id == 6 );
  //assert( 2, min_heap.nodes[2].dist == 0 );

  //debug( "size: %d", min_heap.size );
  //print_min_heap( &min_heap );

  //track_node_t track_graph[TRACK_MAX];
  //init_trackb( track_graph );

  //debug( "track_graph addr: %x", track_graph );
  //debug( "track_node size: %d", sizeof( track_node_t ));

  //debug( "third field address: %x", &(track_graph[0].name));
  //debug( "third field address: %x", &(track_graph[0].type));
  //debug( "third field address: %x", &(track_graph[0].reverse));
  //debug( "third field address: %x", &(track_graph[0].edge));

  //track_node_t * node_3 = &(track_graph[3]);
  //debug( "track_node A4 addr: %x", node_3 );
  //debug( "address diff %d", (node_3 - track_graph) );
  
  track_node_t track_graph[TRACK_MAX];
  init_tracka( track_graph );
  int src_id;
  int dest_id;

  // TESTING dijkstra
  //int all_path[NODE_MAX];
  //int all_dist[NODE_MAX];
  //int all_step[NODE_MAX];

  //src_id = 2;
  //dest_id = 14;

  //dijkstra( track_graph, src_id, all_path, all_dist, all_step );

  //debug( "steps: %d", all_step[dest_id] );
  //int dst_path[all_step[dest_id]];
  
  //debug( "dijkstra results: " );
  //int i;
  //for( i = 0; i < NODE_MAX; ++i ) {
  //  bwprintf( COM2, "node_num: %d, dist: %d, path: %d, step: %d\r\n", 
  //      i, all_dist[i], all_path[i], all_step[i] );
  //}

  //int dst_path[NODE_MAX];
  //int *steps;

  //print_shortest_path( track_graph, all_path, all_step, src_id, dest_id, dst_path );

  //for( i = 0; i < NODE_MAX; ++i ) {
  //  int all_dst_path[all_step[i]];
  //  print_shortest_path( track_graph, all_path, all_step, src_id, i, all_dst_path );
  //}
  //=================================================================================
  
  int switch_states[SW_MAX];
  init_switches( switch_states );

  rail_cmds_t cmds;
  init_rail_cmds( &cmds );
  
  train_state_t train;
  init_trains( &train, track_graph, switch_states );
  train.cur_speed = 12;
  train.speeds[train.cur_speed].safe_branch_distance += 0;
 
  //TESTING switch commands
  //src_id = 2;
  //dest_id = 14;
  //train.prev_sensor_id = src_id;
  //train.dest_id = dest_id;
  //while( train.speeds[train.cur_speed].safe_branch_distance < 500 ) {
  //  init_rail_cmds( &cmds );
  //  //get_next_command( &train, &cmds );
  //  train.speeds[train.cur_speed].safe_branch_distance += 50;

  //debug( "Commandes: \r\nswitch_id0: %d, switch_action0: %d, switch_delay0: %d 
  //                   \n\rswitch_id1: %d, switch_action1: %d, switch_delay1: %d 
  //                   \n\rswitch_id2: %d, switch_action2: %d, switch_delay2: %d",
  //                      cmds.switch_id0, cmds.switch_action0, cmds.switch_delay0, 
  //                      cmds.switch_id1, cmds.switch_action1, cmds.switch_delay1, 
  //                      cmds.switch_id2, cmds.switch_action2, cmds.switch_delay2 );
  //}
  //=================================================================================

  //TESTING reverse commands
  init_rail_cmds( &cmds );
  src_id = 30;
  dest_id = 15;
  int next_sensor_id = 2;
  int i;
  train.prev_sensor_id = src_id;
  train.dest_id= dest_id;
  train.next_sensor_id = next_sensor_id;
  get_next_command( &train, &cmds );
  debug( "Reverse commands: \n\rtrain_id: %d, train_action: %d, train_delay: %d \
                      \r\nswitch_id0: %d, switch_action0: %d, switch_delay0: %d \
                      \n\rswitch_id1: %d, switch_action1: %d, switch_delay1: %d \
                      \n\rswitch_id2: %d, switch_action2: %d, switch_delay2: %d",
                          cmds.train_id, cmds.train_action, cmds.train_delay,
                          cmds.switch_id0, cmds.switch_action0, cmds.switch_delay0, 
                          cmds.switch_id1, cmds.switch_action1, cmds.switch_delay1, 
                          cmds.switch_id2, cmds.switch_action2, cmds.switch_delay2 );
  //=================================================================================

  //TESTING static prediction
  Printf( COM2, "\n\nAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHSW8: %d\r\n", SW8 );
  train.switch_states[SW8] = SW_STRAIGHT;
  predict_next_sensor_static( &train );
  Printf( COM2,  "static next sensor prediction: %d, dist_to_next_sensor: %d\r\n", train.next_sensor_id, train.dist_to_next_sensor );

  train.switch_states[SW8] = SW_CURVED;
  predict_next_sensor_static( &train );
  Printf( COM2, "static next sensor prediction: %d, dist_to_next_sensor: %d\r\n", train.next_sensor_id, train.dist_to_next_sensor );

  Printf( COM2, "Before prediction\r\n" );
  train.prev_sensor_id = src_id;
  train.next_sensor_id = next_sensor_id;
  predict_next_fallback_sensors_static( &train );
  for( i = 0; i < 5; ++i ) {
    if( train.fallback_sensors[i] == -1 ) {
      break;
    }
    Printf( COM2, "fallback_sensor[%d]: %d\r\n", i, train.fallback_sensors[i] );
  }
}

void first_user_task( ){
  debug( "first user task" );
  //bwsetfifo( COM2, OFF );
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

#ifdef A4
  a4_test_task5( );
  
#endif /* A4 */

#ifdef RING_TEST
  ring_buf_test( );
#endif /* TEST */

#ifdef RAIL_TEST
  dijkstra_test( );
#endif /* RAIL_TEST */

  bwprintf( COM2, "Exit first_user_task\n\r");
  Exit( );
}
