#include "global.h"
#include "tools.h"

#include "syscall.h"
#include "nameserver.h"
#include "rail_server.h"
#include "rail_control.h"
#include "rail_helper.h"
#include "calibration.h"
#include "track.h"
#include "io.h"
#include "clock_server.h"
#include "ring_buf.h"

#include "track_data_new.h"
#include "track_node.h"


void rail_graph_worker( ) {
  //DEBUG
  int rail_server_tid = MyParentTid( );
  assert( 1, rail_server_tid > 0 );
  int ret_val;

  /* content to send to the server, first msg has NULL cmds */
  rail_cmds_t rail_cmds;

  /* content to receive from the server */
  train_state_t* train_state; 

  rail_msg_t rail_msg;  
  rail_msg.request_type = RAIL_CMDS;
  rail_msg.to_server_content.rail_cmds = &rail_cmds;
  rail_msg.from_server_content.nullptr= NULL;

  init_rail_cmds( &rail_cmds );

  FOREVER {
    //DEBUG
    ret_val = Send( rail_server_tid, (char*)&rail_msg, sizeof( rail_msg ), (char*)&train_state, sizeof( train_state ));
    assertm( 1, ret_val >= 0, "retval: %d", ret_val );
    assert( 1, train_state );

    init_rail_cmds( &rail_cmds );
    get_next_command( train_state, &rail_cmds );
  //DEBUG
    //TODO: anything else?
  }
}

void sensor_data_courier( ) {
  int rail_server_tid = MyParentTid( );
  assert( 1, rail_server_tid > 0 );
  
  /* content to send to the server */
  sensor_data_t sensor_data;
  //TODO: should zero out for the first msg to avoid data corruption?
  
  rail_msg_t rail_msg;
  rail_msg.request_type = SENSOR_DATA;
  rail_msg.to_server_content.sensor_data = &sensor_data;
  rail_msg.from_server_content.nullptr = NULL; // as lean as possible
  int ret_val;
  int sensor_num;
  int sensor_task_id = WhoIs( (char *) SENSOR_PROCESSING_TASK );
  FOREVER {
    ret_val =Send( sensor_task_id, (char *)&sensor_num, 0, (char *)&sensor_num, sizeof(sensor_num) );
    assertm( 1, ret_val >= 0, "retval: %d", ret_val );
    (rail_msg.to_server_content.sensor_data)->sensor_num = sensor_num;
    ret_val = Send( rail_server_tid, (char*)&rail_msg, sizeof( rail_msg ), (char *)&rail_msg, 0);
    assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  }
  Exit( );
}

void sensor_worker( ) {
  rail_msg_t rail_msg;
  rail_msg.request_type = SENSOR_WORKER_READY;
  rail_msg.to_server_content.train_state = NULL; // train number that just ran over sensor
  rail_msg.from_server_content.nullptr = NULL;
  int ret_val;
  int rail_server_tid;
  int sensor_num;
  int cur_time;
  int expected_train_idx;
  char sensor_name[4];
  train_state_t *trains;
  sensor_args_t sensor_args;
  ret_val = Receive( &rail_server_tid, (char *)&rail_msg, 0 );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  ret_val = Reply( rail_server_tid, (char *)&rail_msg, 0 );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  FOREVER {
    ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&sensor_args, sizeof(sensor_args) );
    assertm( 1, ret_val >= 0, "retval: %d", ret_val );
    trains = sensor_args.trains;
    sensor_num = sensor_args.sensor_num;
    
    // TODO: Change this function to also include fallback sensors
    expected_train_idx = get_expected_train_idx( trains, sensor_num );
    train_state_t *train = expected_train_idx == NONE ? NULL : &( trains[expected_train_idx] ); 
    //assert( 1, train );
    cur_time = Time( );
    if( train == NULL ) {
      continue;
    }
    if( train->state == INITIALIZING ) {
      set_train_speed( train, 0 );
      switch( train->train_id ) {
      case TRAIN_58_NUM:
        init_58( train );
        break;
      case TRAIN_45_NUM:
        init_45( train );
        break;
      case TRAIN_24_NUM:
        init_24( train );
        break;
      case TRAIN_12_NUM:
        init_12( train );
        break;
      default:
        break;
      }
      train->cur_speed = 0;
      train->prev_speed = 0;
      train->speed_change_time = cur_time;
      train->state = READY;
      Printf( COM2, "\0337\033[1A\033[2K\rTrain %d FINISHED INITIALIZING\0338", train->train_id );
      Printf( COM2, "\0337\033[3A\033[2K\rExpected distance to sensor N/A: N/A   \0338" );
      Printf( COM2, "\0337\033[4A\033[2K\rActual distance to sensor N/A: N/A   \0338" );
      Printf( COM2, "\0337\033[5A\033[2K\rDistance difference: N/A   \0338" );
      Printf( COM2, "\0337\033[9A\033[2K\rTime difference: N/A   \0338" );
    } else {
      // Do not update velocity if we have picked up the the train from a fallback sensor
      if( train->next_sensor_id == sensor_num ) {
        update_velocity( train, cur_time, train->time_at_last_landmark, train->dist_to_next_sensor );
        sensor_id_to_name( train->next_sensor_id, sensor_name );
        Printf( COM2, "\0337\033[3A\033[2K\rExpected distance to sensor %c%c%c: %d    \0338", sensor_name[0], sensor_name[1], sensor_name[2], train->mm_past_landmark / 10 );
        Printf( COM2, "\0337\033[4A\033[2K\rActual distance to sensor %c%c%c: %d    \0338", sensor_name[0], sensor_name[1], sensor_name[2], train->dist_to_next_sensor );
        Printf( COM2, "\0337\033[5A\033[2K\rDistance difference: %d    \0338", ( train->mm_past_landmark / 10 ) - train->dist_to_next_sensor );
        Printf( COM2, "\0337\033[9A\033[2K\rTime difference: %d    \0338", train->time_to_next_sensor );
      } else {
        Printf( COM2, "\0337\033[16;30HWOAH NELLY, ALMOST LOST THE TRAIN at time: %d\0338", cur_time );
      }
      train->vel_at_last_landmark = train->cur_vel;
    }
    //assert( 2, train->next_sensor_id == sensor_num );
    rail_msg.to_server_content.train_state = train;
    // updates next_sensor_id and dist_to_next_sensor
    train->time_at_last_landmark = cur_time;
    train->prev_sensor_id = sensor_num;
    train->mm_past_landmark = 0;
    if( train->state == REVERSING ) {
      predict_next_sensor_dynamic( train );
    } else {
      predict_next_sensor_static( train );
    }
    predict_next_fallback_sensors_static( train );
    sensor_id_to_name( train->next_sensor_id, sensor_name );
    Printf( COM2, "\0337\033[7A\033[2K\rNext expected sensor: %c%c%c    \0338", sensor_name[0], sensor_name[1], sensor_name[2] );
    // if no reverse, 
  }
}

void train_exe_worker( ) {
  rail_msg_t rail_msg;
  rail_msg.request_type = TRAIN_EXE_READY;
  rail_msg.to_server_content.nullptr = NULL;
  rail_msg.from_server_content.nullptr = NULL;
  int ret_val;
  int rail_server_tid;
  train_state_t *train;
  train_cmd_args_t train_cmd_args;
  ret_val = Receive( &rail_server_tid, (char *)&train, sizeof( train ) );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  ret_val = Reply( rail_server_tid, (char *)&rail_msg, 0 );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );

  FOREVER {
    ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&train_cmd_args, sizeof(train_cmd_args) );
    assertm( 1, ret_val >= 0, "retval: %d", ret_val );
    if( train_cmd_args.delay_time > 0 ) {
      Delay( train_cmd_args.delay_time );
    }
    switch( train_cmd_args.cmd ) {
    case TR_STOP:
      train->state = STOPPING;
      set_train_speed( train, 0 );
      train->state = READY;
      break;
    case TR_REVERSE:
      {
        train->state = REVERSING;
        int prev_speed = train->cur_speed;
        int stopping_time = get_cur_stopping_time( train ) / 10;
        set_train_speed( train, 0 );
        Delay( stopping_time + STOP_TIME_BUFFER ); // TODO: Change this to stopping time;
        set_train_speed( train, 15 );
        set_train_speed( train, prev_speed );
        train->state = READY;
        break;
      }
    case TR_CHANGE_SPEED:
      train->state = BUSY;
      set_train_speed( train, train_cmd_args.speed_num );
      train->state = READY;
      // rerun graph search / prediction
      break;
    case TR_INIT:
      train->state = INITIALIZING;
      set_train_speed( train, INIT_SPEED );
      break;
    case TR_DEST:
      {
        train->dest_id = train_cmd_args.dest;
        train->mm_past_dest = train_cmd_args.mm_past_dest;
        char dest[3];
        sensor_id_to_name( train->dest_id, dest );
        if( train->dest_id == -1 ) {
          Printf( COM2, "\0337\033[9A\033[2K\rCurrent destination: N/A\0338" );
        } else {
          Printf( COM2, "\0337\033[9A\033[2K\rCurrent destination: %c%c%c\0338", dest[0], dest[1], dest[2] );
        }
        break;
      }
    case TR_ACCEL:
      train->accel_rate = train_cmd_args.accel_rate;
      break;
    case TR_DECEL:
      train->decel_rate = train_cmd_args.decel_rate;
      break;
    case TR_CH_DIR:
      train->is_forward ^= 1;
      if( train->is_forward ) {
        Printf( COM2, "\0337\033[1A\033[2K\rTrain %d now facing forwards\0338", train->train_id );
      } else {
        Printf( COM2, "\0337\033[1A\033[2K\rTrain %d now facing backwards\0338", train->train_id );
      }
      break;
    default:
      break;
    }
    // rerun prediction
  }
}

void switch_exe_worker( ) {
  rail_msg_t rail_msg;
  rail_msg.request_type = SWITCH_EXE_READY;
  rail_msg.to_server_content.nullptr = NULL;
  rail_msg.from_server_content.nullptr = NULL;
  int ret_val;
  int rail_server_tid;
  int switch_num;
  int state;
  int *switch_states;
  switch_cmd_args_t switch_cmd_args;
  ret_val = Receive( &rail_server_tid, (char *)&switch_num, sizeof( switch_num ) );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  ret_val = Reply( rail_server_tid, (char *)&rail_msg, 0 );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );

  FOREVER {
    ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&switch_cmd_args, sizeof(switch_cmd_args) );
    assertm( 1, ret_val >= 0, "retval: %d", ret_val );
    if( switch_cmd_args.delay_time > 0 ) {
      ret_val = Delay( switch_cmd_args.delay_time );
      assertm( 1, ret_val >= 0, "retval: %d", ret_val );
    }
    state = switch_cmd_args.state;
    switch_states = switch_cmd_args.switch_states;
    switch( state ) {
    case SW_STRAIGHT:
      set_switch( switch_num, STRAIGHT, switch_states );
      break;
    case SW_CURVED:
      set_switch( switch_num, CURVED, switch_states );
      break;
    default:
      break;
    }
  }
}

void update_trains( ) {
  train_state_t *trains;
  int ret_val;
  int rail_server_tid;
  update_train_args_t update_train_args;
  ret_val = Receive( &rail_server_tid, (char *)&update_train_args, sizeof( update_train_args ) );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  ret_val = Reply( rail_server_tid, (char *)&trains, 0 );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  trains = update_train_args.trains;
  int i;
  int cur_time;
  Printf( COM2, "\0337\033[6A\033[2K\rmm past last sensor: N/A\0338" );
  Printf( COM2, "\0337\033[8A\033[2K\rCurrent velocity: N/A\0338" );
  Printf( COM2, "\0337\033[10A\033[2K\rTime to next sensor: N/A\0338" );
  FOREVER {
    cur_time = Delay( 1 );
    for( i = 0; i < TR_MAX; ++i ) {
      if( trains[i].state != NOT_INITIALIZED ) {
        trains[i].cur_vel = get_cur_velocity( &(trains[i]), cur_time );
        trains[i].mm_past_landmark = get_mm_past_last_landmark( &(trains[i]), cur_time );
        trains[i].time_since_last_pos_update = cur_time;
        trains[i].vel_at_last_pos_update = trains[i].cur_vel;
        trains[i].time_to_next_sensor = time_to_node( &(trains[i]), trains[i].dist_to_next_sensor, cur_time );
        if( cur_time % 10 == 0 ) {
          Printf( COM2, "\0337\033[18;22H%d    \0338", trains[i].mm_past_landmark / 10 );
          Printf( COM2, "\0337\033[16;19H%d    \0338", trains[i].cur_vel );
          Printf( COM2, "\0337\033[14;22H%d    \0338", trains[i].time_to_next_sensor );
        }
      }
    }
  }
}

void rail_server( ) {
  if( RegisterAs( (char*)RAIL_SERVER ) == -1 ) {
    bwputstr( COM2, "ERROR: failed to register rail_server, aborting ...\n\r" );
    Exit( );
  }
  int client_tid;
  int ret_val;
  
  /* track state initialization */
  track_node_t track_graph[TRACK_MAX];
  init_tracka( (track_node_t*)track_graph );
  int switch_states[SW_MAX];
  rail_msg_t receive_msg;

  /* trains initialization */
  train_state_t trains[TR_MAX];
  init_trains( trains, (track_node_t*)track_graph, switch_states );
  init_switches( switch_states );
  int rail_graph_worker_tids[TR_MAX]; 

  //bool train_graph_search_ready[TR_MAX];
  //bool train_delay_treads_arrived[TR_MAX]; TODO
  /* graph_search_workers */
  int i;
  for( i = 0; i < TR_MAX; ++i ) {
    rail_graph_worker_tids[i] = Create( 10, &rail_graph_worker );
    assert( 1, rail_graph_worker_tids[i] > 0 );
  }
  rail_cmds_t* recved_cmds; 


  int update_trains_tid = Create( 13, &update_trains );
  update_train_args_t update_train_args;
  update_train_args.trains = trains;
  ret_val = Send( update_trains_tid, (char *)&update_train_args, sizeof( update_train_args ), (char *)&client_tid, 0 );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );


  int worker_tid;
  /* Sensor worker declarations */
  int sensor_courier_tid = Create( 2, &sensor_data_courier );
  assert( 1, sensor_courier_tid > 0 );

  declare_ring_queue( int, sensor_workers, SENSOR_WORKER_MAX );
  int sensor_worker_tids[SENSOR_WORKER_MAX];
  char sensor_name[4];
  for( i = 0; i < SENSOR_WORKER_MAX; ++i ) {
    sensor_worker_tids[i] = Create( 10, &sensor_worker );
    assert( 1, sensor_worker_tids[i] > 0 );
    ret_val = Send( sensor_worker_tids[i], (char *)&client_tid, 0, (char *)&client_tid, 0 );
    assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  }
  sensor_args_t sensor_args;
  sensor_args.trains = trains;

  /* Train action workers */
  int train_exe_worker_tids[TR_MAX];
  int train_exe_worker_tid = NONE;
  for( i = 0; i < TR_MAX; ++i ) {
    train_exe_worker_tids[i] = Create( 10, &train_exe_worker );
    assert( 1, train_exe_worker_tids[i] > 0 );
    train_state_t *train = &(trains[i]);
    ret_val = Send( train_exe_worker_tids[i], (char *)&train, sizeof( train ), (char *)&client_tid, 0 );
    assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  }
  train_cmd_args_t train_cmd_args;

  /* Switch action workers */
  int switch_exe_worker_tid = NONE;
  int switch_exe_worker_tids[SW_MAX];
  int switch_num;
  for( i = 1; i < SW_MAX; ++i ) {
    switch_exe_worker_tids[i] = Create( 10, &switch_exe_worker );
    assert( 1, switch_exe_worker_tids[i] > 0 );
    switch_num = i;
    if( switch_num > 18 ) {
      switch_num += 134;
    }
    ret_val = Send( switch_exe_worker_tids[i], (char *)&switch_num, sizeof( switch_num ), (char *)&client_tid, 0 );
    assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  }
  switch_cmd_args_t switch_cmd_args;
  switch_cmd_args.switch_states = switch_states;

  FOREVER { 
    ret_val = Receive( &client_tid, (char *)&receive_msg, sizeof( rail_msg_t ));
    assertm( 1, ret_val >= 0, "retval: %d", ret_val );
    switch( receive_msg.request_type ) {
      case SENSOR_DATA:
        {
        //DEBUG
          /* retrieve data, find sensor number and corresponding train, set ready */
          ret_val = Reply( client_tid, (char *)&receive_msg, 0 );
          assertm( 1, ret_val >= 0, "retval: %d", ret_val );
          if( !sensor_workers_empty( ) ) {
            int sensor_num = (receive_msg.to_server_content.sensor_data)->sensor_num;
            worker_tid = sensor_workers_pop_front( );
            sensor_args.sensor_num = sensor_num;
            ret_val = Reply( worker_tid, (char *)&sensor_args, sizeof(sensor_args) );
            assertm( 1, ret_val >= 0, "retval: %d", ret_val );
          }
        }
        break;
      case USER_INPUT:
        //DEBUG
        ret_val = Reply( client_tid, (char *)&receive_msg, 0 );
        assertm( 1, ret_val >= 0, "retval: %d", ret_val );
        if( (receive_msg.to_server_content.rail_cmds)->train_id ) {
          // TODO: Make a mapping between train number and idx
          switch( (receive_msg.to_server_content.rail_cmds)->train_id ) {
          case TRAIN_58_NUM:
            train_exe_worker_tid = train_exe_worker_tids[TRAIN_58_IDX];
            break;
          case TRAIN_45_NUM:
            train_exe_worker_tid = train_exe_worker_tids[TRAIN_45_IDX];
            break;
          case TRAIN_24_NUM:
            train_exe_worker_tid = train_exe_worker_tids[TRAIN_24_IDX];
            break;
          case TRAIN_12_NUM:
            train_exe_worker_tid = train_exe_worker_tids[TRAIN_12_IDX];
            break;
          default:
            train_exe_worker_tid = -1;
            break;
          }
          if( train_exe_worker_tid > 0 ) {
            train_cmd_args.cmd = (receive_msg.to_server_content.rail_cmds)->train_action;
            train_cmd_args.speed_num = (receive_msg.to_server_content.rail_cmds)->train_speed;
            train_cmd_args.delay_time = (receive_msg.to_server_content.rail_cmds)->train_delay;
            train_cmd_args.dest = (receive_msg.to_server_content.rail_cmds)->train_dest;
            train_cmd_args.mm_past_dest = (receive_msg.to_server_content.rail_cmds)->train_mm_past_dest;
            train_cmd_args.accel_rate = (receive_msg.to_server_content.rail_cmds)->train_accel;
            train_cmd_args.decel_rate = (receive_msg.to_server_content.rail_cmds)->train_decel;
            ret_val = Reply( train_exe_worker_tid, (char *)&train_cmd_args, sizeof( train_cmd_args ) );
            //FIXME: if the train_exe_worker_tid is not ready, should not Reply, or add a secretary
            assertm( 1, ret_val == 0, "ret_val: %d", ret_val );
          }
        } else if ( (receive_msg.to_server_content.rail_cmds)->switch_id0 ) {
          int switch_id = (receive_msg.to_server_content.rail_cmds)->switch_id0;
          CONVERT_SWITCH_ID( switch_id );
          int switch_exe_worker_tid = switch_exe_worker_tids[switch_id];
          switch_cmd_args.state = (receive_msg.to_server_content.rail_cmds)->switch_action0;
          switch_cmd_args.delay_time = 0;
          ret_val = Reply( switch_exe_worker_tid, (char *)&switch_cmd_args, sizeof( switch_cmd_args ) );
          assertm( 1, ret_val >= 0, "retval: %d", ret_val );
        }
        break;
      case RAIL_CMDS:
        //DEBUG:
        /* get and send train cmds */
        recved_cmds = receive_msg.to_server_content.rail_cmds;
        switch( (receive_msg.to_server_content.rail_cmds)->train_id ) {
        case TRAIN_58_NUM:
          train_exe_worker_tid = train_exe_worker_tids[TRAIN_58_IDX];
          break;
        case TRAIN_45_NUM:
          train_exe_worker_tid = train_exe_worker_tids[TRAIN_45_IDX];
          break;
        case TRAIN_24_NUM:
          train_exe_worker_tid = train_exe_worker_tids[TRAIN_24_IDX];
          break;
        case TRAIN_12_NUM:
          train_exe_worker_tid = train_exe_worker_tids[TRAIN_12_IDX];
          break;
        default:
          train_exe_worker_tid = -1;
          break;
        }
        if( train_exe_worker_tid > 0 ) {
          train_cmd_args.cmd = recved_cmds->train_action;
          train_cmd_args.speed_num = recved_cmds->train_speed;
          train_cmd_args.delay_time = recved_cmds->train_delay;
          ret_val = Reply( train_exe_worker_tid, (char*)&train_cmd_args, sizeof( train_cmd_args ));
          assertm( 1, ret_val >= 0, "retval: %d", ret_val );
        }
        //FIXME: make this into a function or macro
        /* get and send switch cmds */
        if( recved_cmds->switch_id0 != NONE ) {
          switch_exe_worker_tid = switch_exe_worker_tids[recved_cmds->switch_id0];
          switch_cmd_args.state = recved_cmds->switch_action0;
          switch_cmd_args.delay_time = recved_cmds->switch_delay0;
          ret_val = Reply( switch_exe_worker_tid, (char*)&switch_cmd_args, sizeof( switch_cmd_args ));
          assertm( 1, ret_val >= 0, "retval: %d, switch_tid: %d, switch_id: %d", ret_val, 
              switch_exe_worker_tid, recved_cmds->switch_id0 );
        }
        if( recved_cmds->switch_id1 != NONE ) {
          switch_exe_worker_tid = switch_exe_worker_tids[recved_cmds->switch_id1];
          switch_cmd_args.state = recved_cmds->switch_action1;
          switch_cmd_args.delay_time = recved_cmds->switch_delay1;
          ret_val = Reply( switch_exe_worker_tid, (char*)&switch_cmd_args, sizeof( switch_cmd_args ));
          assertm( 1, ret_val >= 0, "retval: %d", ret_val );
        }
        if( recved_cmds->switch_id2 != NONE ) {
          switch_exe_worker_tid = switch_exe_worker_tids[recved_cmds->switch_id2];
          switch_cmd_args.state = recved_cmds->switch_action2;
          switch_cmd_args.delay_time = recved_cmds->switch_delay2;
          ret_val = Reply( switch_exe_worker_tid, (char*)&switch_cmd_args, sizeof( switch_cmd_args ));
          assertm( 1, ret_val >= 0, "retval: %d", ret_val );
        }
        if( recved_cmds->switch_id3 != NONE ) {
          switch_exe_worker_tid = switch_exe_worker_tids[recved_cmds->switch_id3];
          switch_cmd_args.state = recved_cmds->switch_action3;
          switch_cmd_args.delay_time = recved_cmds->switch_delay3;
          ret_val = Reply( switch_exe_worker_tid, (char*)&switch_cmd_args, sizeof( switch_cmd_args ));
          assertm( 1, ret_val >= 0, "retval: %d", ret_val );
        }
        break;
      case TRAIN_EXE_READY:
        //DEBUG
        // rerun graph search only if command was given by user AND if command is reverse or change speed
        // Don't run graph search if train state is busy, or reversing
        // Update train direction if reverse is given
        break;
      case SWITCH_EXE_READY:
        //DEBUG
        for( i = 0; i < TR_MAX; ++i ) {
          if( trains[i].state != NOT_INITIALIZED ) {
            if( trains[i].state == REVERSING ) {
              predict_next_sensor_dynamic( &(trains[i]) );
            } else {
              predict_next_sensor_static( &(trains[i]) );
            }
            predict_next_fallback_sensors_static( &(trains[i]) );
            sensor_id_to_name( trains[i].next_sensor_id, sensor_name );
            Printf( COM2, "\0337\033[7A\033[2K\rNext expected sensor: %c%c%c    \0338", sensor_name[0], sensor_name[1], sensor_name[2] );
          }
        }
        break;
      case SENSOR_WORKER_READY: // worker responds with the train that hit the sensor
        // get train, graph search
        sensor_workers_push_back( client_tid );
        if( receive_msg.to_server_content.train_state != NULL ) {
          // FIXME: reply to the trigger the first
          // NOTE: Does the above matter? Reply is non-blocking. We're saving only a few nanoseconds by doing so.
          int i = 0;
          for( ;  i < TR_MAX ; ++i ) {
            if( trains[i].dest_id != NONE && trains[i].state != REVERSING ) {
              train_state_t * train_to_send = &(trains[i]);
              ret_val = Reply( rail_graph_worker_tids[i], (char*)&( train_to_send ), sizeof( train_to_send ));
              assertm( 1, ret_val == 0, "ret_val: %d, tid: %d", ret_val, rail_graph_worker_tids[i] );
            }
          }
        }
        //TODO: run dynamic graph search
        break;
      default:
        assertm( 1, false, "ERROR: unrecognized request: %d", receive_msg.request_type );
        break;
    }
  }
}
