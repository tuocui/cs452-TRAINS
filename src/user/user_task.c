#include <tools.h>
#include <user_task.h>
#include <syscall.h>
#include <nameserver.h>
#include <rps.h>
#include "clock_server.h"
#include "io.h"
#include "ring_buf.h"

#define CYCLES 1000

// TODO: Split this into user task and test task files

//#define A1 1
//#define A2 1 
//#define A3 1
//#define A4 1
#define TEST

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

#ifdef A4

void parse_user_input( ) {
  char d;
  char e;
  char c[3];
  c[0] = 'a';
  c[1] = 'b';
  c[2] = 'c';
  FOREVER {
    d = (char)Getc( COM2 );
    e = (char)Getc( COM2 );
    c[0] = d;
    c[1] = e;
    Putstr( COM2, c, 2 );    
    Putc( COM2, e );
  }
  Putstr( COM2, c, 3);
}

void track_sensor_task( ) {
  char c;
  int module_num = 0;
	int sensor_num = 1;
	int i = 0;
  int j = 0;
  int recent_sensor;
  int most_recent_sensor = 0;
  int recent_sensors[NUM_RECENT_SENSORS];
  int recent_sensors_ind = 0;
  int num_sensors_triggered = 0;
  int recent_sensor_ind;
  char module_num_c;
  int recent_sensor_triggered = 0;
  FOREVER {
    Putc( COM1, REQUEST_SENSOR );
	  module_num = 0;
    recent_sensor_triggered = 0;
    for( i = 0; i < NUM_SENSOR_BYTES; ++i ) {
      c = (char) Getc( COM1 );
      //bwprintf( COM2, "got char: %x, module_num: %d\r\n", c, module_num );
      sensor_num = 1;
	    if ( c > 0 ) {
	      if ( module_num % 2 == 1 ) {
		      sensor_num += 8;
	      }
	      for( j = 0; j < 8 ; ++j ) {
		      // Yay for bitwise operations
		      if ( ( c >> ( 7 - j ) ) & 0x1 ) {
	          recent_sensor = ( module_num * 100 ) + sensor_num;
	          if ( recent_sensor == most_recent_sensor ) {
		          ++sensor_num;
		          continue;
	          }
            recent_sensor_triggered = 1;
	          recent_sensors[recent_sensors_ind] = recent_sensor;
	          ++num_sensors_triggered;
	          most_recent_sensor = recent_sensor;
	          recent_sensors_ind = ( recent_sensors_ind + 1 ) % NUM_RECENT_SENSORS;
		      }
		      ++sensor_num;
	      }
	    }
	    ++module_num;
    }

    recent_sensor_ind = recent_sensors_ind - 1;
    for( j = 0 ; recent_sensor_triggered && j < NUM_RECENT_SENSORS && j < num_sensors_triggered; ++j ) {
	    if ( recent_sensor_ind == -1 ) recent_sensor_ind = NUM_RECENT_SENSORS - 1;
	    recent_sensor = recent_sensors[recent_sensor_ind];
	    sensor_num = recent_sensor % 100;
	    module_num = recent_sensor / 100;
	    module_num_c = ( ( char ) ( module_num / 2 ) ) + 'A';
	    bwprintf( COM2, "    %c%d  \r\n", module_num_c, sensor_num );
	    --recent_sensor_ind;
    }
    //Delay( 10 );
  }
  Exit( );
}

void a4_test_task( ) {
  setspeed( COM1, LOW_SPEED );
  wait_cycles(1000);
  enable_two_stop_bits( COM1 );
  setfifo( COM1, OFF );
  int nameserver_tid = Create( 3, &nameserver_main );
  debug( "Nameserver tid: %d", nameserver_tid );
  int idle_id = Create( PRIORITY_MAX, &idle_task );
  debug( "Idle tid: %d", idle_id );
  int com1_out_server_tid = Create( 3, &COM1_Out_Server );
  debug( "COM1_Out Server tid: %d", com1_out_server_tid );
  int com1_in_server_tid = Create( 3, &COM1_In_Server );
  debug( "COM1_In Server tid: %d", com1_in_server_tid );
  // Create clock server
  int clock_server_tid = Create( 3, &clock_server );
  debug( "Clock Server tid: %d", clock_server_tid );
  char msg[4];
  msg[0] = 13;
  msg[1] = 54;
  msg[2] = 13;
  msg[3] = 58;
  Putstr( COM1, msg, 4 );
  int track_sensor_task_tid = Create( 6, &track_sensor_task );
  debug( "Track Sensor task tid: %d", track_sensor_task_tid );
  Exit( );
}

void a4_test_task2( ) {
  setfifo( COM2, OFF );
  int nameserver_tid = Create( 3, &nameserver_main );
  debug( "Nameserver tid: %d", nameserver_tid );
  int idle_id = Create( PRIORITY_MAX, &idle_task );
  debug( "Idle tid: %d", idle_id );
  int com2_out_server_tid = Create( 3, &COM2_Out_Server );
  debug( "COM1_Out Server tid: %d", com2_out_server_tid );
  int com2_in_server_tid = Create( 3, &COM2_In_Server );
  debug( "COM1_In Server tid: %d", com2_in_server_tid );
  // Create clock server
  int clock_server_tid = Create( 3, &clock_server );
  debug( "Clock Server tid: %d", clock_server_tid );
  int parse_user_input_tid = Create( 6, &parse_user_input );
  debug( "Track Sensor task tid: %d", parse_user_input_tid );
  Exit( );
}
#endif /* A4 */

void ring_buf_test( ) {
  declare_ring_queue(int, test, 2 );
  debug( "buf_count: %d", test_count( ) );
  int idx;
  idx = test_push_front( 1 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );

  idx = test_push_front( 2 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );

  idx = test_push_front( 3 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );
  
  idx = test_push_front( 4 );
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

  idx = test_push_front( 12 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );

  idx = test_push_front( 13 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );
  
  idx = test_push_front( 14 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );

  idx = test_push_front( 15 );
  debug( "buf_count: %d, idx: %d", test_count( ), idx );

  declare_ring_queue( char, com2_buf, 3 );
  debug( "buf_count: %d", com2_buf_count( ) );
  idx = com2_buf_push_front( 'a' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  idx = com2_buf_push_front( 'b' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );
  
  idx = com2_buf_push_front( 'c' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  idx = com2_buf_push_front( 'd' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  char c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  idx = com2_buf_push_front( 'e' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );
  
  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  idx = com2_buf_push_front( 'f' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  idx = com2_buf_push_front( 'g' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  idx = com2_buf_push_front( 'h' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );

  idx = com2_buf_push_front( 'i' );
  debug( "buf_count: %d, idx: %d", com2_buf_count( ), idx );
  
  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  c = com2_buf_pop_back( );
  debug( "buf_count: %d, char: %c", com2_buf_count( ), c );

  idx = com2_buf_push_front( 'j' );
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
  idx = st_buf_push_front( &st1 );
  debug( "buf_count: %d, idx: %d", st_buf_count( ), idx );

  idx = st_buf_push_front( &st2 );
  debug( "buf_count: %d, idx: %d", st_buf_count( ), idx );

  idx = st_buf_push_front( &st3 );
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

void first_user_task( ){
  bwsetfifo( COM2, OFF );
  debug( "first user task" );
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
  a4_test_task2( );
  
#endif /* A4 */

#ifdef TEST
  ring_buf_test( );
#endif /* TEST */

  bwprintf( COM2, "Exit first_user_task\n\r");
  Exit( );
}
