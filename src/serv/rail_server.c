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


/* a notifier, will change to couriers if necessary */
//NOTE: each train should have its own graph_search thread, becuase cmds is local
// and is not synchronized
void rail_graph_worker( ) {
  int rail_server_tid = MyParentTid( );
  assert( 1, rail_server_tid > 0 );

  /* content to send to the server, first msg has NULL cmds */
  rail_cmds_t rail_cmds;
  init_rail_cmds( &rail_cmds ); 

  /* content to receive from the server */
  train_state_t train_state; 

  rail_msg_t rail_msg;  
  rail_msg.request_type = RAIL_CMDS;
  rail_msg.to_server_content.rail_cmds = &rail_cmds;
  rail_msg.from_server_content.train_state = &train_state;

  FOREVER {
    Send( rail_server_tid, (char*)&rail_msg, sizeof( rail_msg ), 
                           (char*)&rail_msg, sizeof( rail_msg ));
    assert( 1, rail_msg.from_server_content.train_state );

    get_next_command( &train_state, &rail_cmds );
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
  int sensor_num;
  int sensor_task_id = WhoIs( (char *) SENSOR_PROCESSING_TASK );
  FOREVER {
    Send( sensor_task_id, (char *)&sensor_num, 0, (char *)&sensor_num, sizeof(sensor_num) );
    (rail_msg.to_server_content.sensor_data)->sensor_num = sensor_num;
    Send( rail_server_tid, (char*)&rail_msg, sizeof( rail_msg ), (char *)&rail_msg, 0);
  }
  Exit( );
}

void sensor_worker( ) {
  rail_msg_t rail_msg;
  rail_msg.request_type = SENSOR_WORKER_READY;
  rail_msg.to_server_content.train_state = NULL; // train number that just ran over sensor
  rail_msg.from_server_content.nullptr = NULL;
  int rail_server_tid;
  int sensor_num;
  int cur_time;
  train_state_t *trains;
  sensor_args_t sensor_args;
  Receive( &rail_server_tid, (char *)&rail_msg, 0 );
  Reply( rail_server_tid, (char *)&rail_msg, 0 );
  FOREVER {
    Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&sensor_args, sizeof(sensor_args) );
    sensor_num = sensor_args.sensor_num;
    trains = sensor_args.trains;
    // TODO: WILSON, Figure out which train this is. If this sensor doesn't match a train, check whether or not there's a train initializing. Otherwise, a random sensor just went off.
    // 1. loop through the train structs, return train_state_*, direct function call
    // 2. if can't find matching train, go through to check INITIALIZIN state,
    // 3. else, return NULL, maybe remember it for later user
    train_state_t *train = &(trains[TRAIN_58]); 
    assert( 1, train );

    cur_time = Time( );
    if( train->state == INITIALIZING ) {
      set_train_speed( train, 0 );
      train->cur_speed = 0;
      train->prev_speed = 0;
      train->speed_change_time = cur_time;
    } else {
      train->vel_at_last_landmark = train->cur_vel;
      //update_velocity( train, cur_time, train->time_at_last_landmark, train->dist_to_next_sensor );
    }
    //assert( 2, train->next_sensor_id == sensor_num );
    train->prev_sensor_id = sensor_num;
    predict_next_sensor_static( train ); 
    // if no reverse, 
    train->time_at_last_landmark = cur_time;
    train->mm_past_landmark = 0;
    rail_msg.to_server_content.train_state = train;
  }
}

void train_exe_worker( ) {
  rail_msg_t rail_msg;
  rail_msg.request_type = TRAIN_EXE_READY;
  rail_msg.to_server_content.nullptr = NULL;
  rail_msg.from_server_content.nullptr = NULL;
  int rail_server_tid;
  train_state_t *train;
  train_cmd_args_t train_cmd_args;
  Receive( &rail_server_tid, (char *)&train, sizeof( train ) );
  Reply( rail_server_tid, (char *)&rail_msg, 0 );

  FOREVER {
    Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&train_cmd_args, sizeof(train_cmd_args) );
    if( train_cmd_args.delay_time > 0 ) {
      Delay( train_cmd_args.delay_time );
    }
    switch( train_cmd_args.cmd ) {
    case TR_STOP:
      train->state = STOPPING;
      set_train_speed( train, 0 );
      break;
    case TR_REVERSE:
      {
        train->state = REVERSING;
        int prev_speed = train->cur_speed;
        set_train_speed( train, 0 );
        Delay( 350 ); // TODO: Change this to stopping time;
        set_train_speed( train, 15 );
        set_train_speed( train, prev_speed );
        break;
      }
    case TR_CHANGE_SPEED:
      train->state = BUSY;
      set_train_speed( train, train_cmd_args.speed_num );
      // rerun graph search / prediction
      break;
    default:
      break;
    }
    train->state = READY;
    // rerun prediction
  }
}

void switch_exe_worker( ) {
  rail_msg_t rail_msg;
  rail_msg.request_type = SWITCH_EXE_READY;
  rail_msg.to_server_content.nullptr = NULL;
  rail_msg.from_server_content.nullptr = NULL;
  int rail_server_tid;
  int switch_num;
  int state;
  int *switch_states;
  switch_cmd_args_t switch_cmd_args;
  Receive( &rail_server_tid, (char *)&switch_num, sizeof( switch_num ) );
  Reply( rail_server_tid, (char *)&rail_msg, 0 );

  FOREVER {
    Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&switch_cmd_args, sizeof(switch_cmd_args) );
    if( switch_cmd_args.delay_time > 0 ) {
      Delay( switch_cmd_args.delay_time );
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
  int rail_server_tid;
  update_train_args_t update_train_args;
  Receive( &rail_server_tid, (char *)&update_train_args, sizeof( update_train_args ) );
  Reply( rail_server_tid, (char *)&trains, 0 );
  trains = update_train_args.trains;
  int i;
  int cur_time;
  int last_mm_past_landmark = 0;
  FOREVER {
    cur_time = Delay( 1 );
    for( i = 0; i < TR_MAX; ++i ) {
      trains[i].mm_past_landmark = get_mm_past_last_landmark( &(trains[i]), cur_time );
      if( trains[i].mm_past_landmark < last_mm_past_landmark ) {
        Printf( COM2, "\0337\033[1A\033[2K\rdistance between sensors: %d\0338", last_mm_past_landmark );
        Printf( COM2, "\0337\033[2A\033[2K\rTrain %d, cur_vel: %d    \0338", trains[i].train_id, trains[i].cur_vel );
        Printf( COM2, "\0337\033[2A\033[2K\rTrain %d, stopping dist: %d    \0338", trains[i].train_id, get_cur_stopping_distance( &(trains[i] ) ) );
      }
      last_mm_past_landmark = trains[i].mm_past_landmark;
      trains[i].cur_vel = get_cur_velocity( &(trains[i]), cur_time );
    }
  }
}

void rail_server( ) {
  if( RegisterAs( (char*)RAIL_SERVER ) == -1 ) {
    bwputstr( COM2, "ERROR: failed to register rail_server, aborting ...\n\r" );
    Exit( );
  }
  int client_tid;
  
  /* track state initialization */
  track_node_t track_graph[TRACK_MAX];
  init_trackb( (track_node_t*)track_graph );
  int switch_states[SW_MAX];
  int train_graph_search_tid[TR_MAX]; 

  
  /* trains initialization */
  train_state_t trains[TR_MAX];
  init_trains( trains, (track_node_t*)track_graph, switch_states );
  init_switches( switch_states );

  //bool train_graph_search_ready[TR_MAX];
  //bool train_delay_treads_arrived[TR_MAX]; TODO
  rail_msg_t receive_msg;
  int i;
  for( i = 0; i < TR_MAX; ++i ) {
    train_graph_search_tid[i] = Create( 10, &rail_graph_worker );
    assert( 1, train_graph_search_tid[i] > 0 );
  }
  
  int sensor_courier_tid = Create( 2, &sensor_data_courier );
  assert( 1, sensor_courier_tid > 0 );

  //TODO: add secretary/courier

  int update_trains_tid = Create( 13, &update_trains );
  update_train_args_t update_train_args;
  update_train_args.trains = trains;
  Send( update_trains_tid, (char *)&update_train_args, sizeof( update_train_args ), (char *)&client_tid, 0 );

  int worker_tid;
  /* Sensor worker declarations */
  declare_ring_queue( int, sensor_workers, SENSOR_WORKER_MAX );
  int sensor_worker_tids[SENSOR_WORKER_MAX];
  for( i = 0; i < SENSOR_WORKER_MAX; ++i ) {
    sensor_worker_tids[i] = Create( 10, &sensor_worker );
    assert( 1, sensor_worker_tids[i] > 0 );
    Send( sensor_worker_tids[i], (char *)&client_tid, 0, (char *)&client_tid, 0 );
  }
  sensor_args_t sensor_args;
  sensor_args.trains = trains;

  /* Train action workers */
  int train_exe_worker_tids[TR_MAX];
  for( i = 0; i < TR_MAX; ++i ) {
    train_exe_worker_tids[i] = Create( 10, &train_exe_worker );
    assert( 1, train_exe_worker_tids[i] > 0 );
    train_state_t *train = &(trains[i]);
    Send( train_exe_worker_tids[i], (char *)&train, sizeof( train ), (char *)&client_tid, 0 );
  }
  train_cmd_args_t train_cmd_args;

  /* Switch action workers */
  int switch_exe_worker_tids[SW_MAX];
  for( i = 1; i < SW_MAX; ++i ) {
    switch_exe_worker_tids[i] = Create( 10, &switch_exe_worker );
    assert( 1, switch_exe_worker_tids[i] > 0 );
    Send( switch_exe_worker_tids[i], (char *)&i, sizeof( i ), (char *)&client_tid, 0 );
  }
  switch_cmd_args_t switch_cmd_args;
  switch_cmd_args.switch_states = switch_states;

  FOREVER { 
    Receive( &client_tid, (char *)&receive_msg, sizeof( rail_msg_t ));
    switch( receive_msg.request_type ) {
      case SENSOR_DATA:
        {
          /* retrieve data, find sensor number and corresponding train, set ready */
          // prediction on when it will hit next landmark.
          Reply( client_tid, (char *)&receive_msg, 0 );
          if( !sensor_workers_empty( ) ) {
            int sensor_num = (receive_msg.to_server_content.sensor_data)->sensor_num;
            worker_tid = sensor_workers_pop_front( );
            sensor_args.sensor_num = sensor_num;
            Reply( worker_tid, (char *)&sensor_args, sizeof(sensor_args) );
          }
        }
        break;
      case USER_INPUT:
        Reply( client_tid, (char *)&receive_msg, 0 );
        if( (receive_msg.to_server_content.rail_cmds)->train_id ) {
          // TODO: Make a mapping between train number and idx
          int train_exe_worker_tid;
          switch( (receive_msg.to_server_content.rail_cmds)->train_id ) {
          case 58:
            train_exe_worker_tid = train_exe_worker_tids[TRAIN_58];
            train_cmd_args.cmd = (receive_msg.to_server_content.rail_cmds)->train_action;
            train_cmd_args.speed_num = (receive_msg.to_server_content.rail_cmds)->train_speed;
            train_cmd_args.delay_time = (receive_msg.to_server_content.rail_cmds)->train_delay;
            Reply( train_exe_worker_tid, (char *)&train_cmd_args, sizeof( train_cmd_args ) );
            break;
          default:
            break;
          }
        } else if ( (receive_msg.to_server_content.rail_cmds)->switch_id0 ) {
          int switch_exe_worker_tid = switch_exe_worker_tids[(receive_msg.to_server_content.rail_cmds)->switch_id0];
          switch_cmd_args.state = (receive_msg.to_server_content.rail_cmds)->switch_action0;
          switch_cmd_args.delay_time = 0;
          Reply( switch_exe_worker_tid, (char *)&switch_cmd_args, sizeof( switch_cmd_args ) );
        }
        break;
      case RAIL_CMDS:
        // get train state
        // do some stuff
        break;
      case TRAIN_EXE_READY:
        break;
      case SWITCH_EXE_READY:
        break;
      case SENSOR_WORKER_READY: // worker responds with the train that hit the sensor
        // get train, graph search
        sensor_workers_push_back( client_tid );
        if( receive_msg.to_server_content.train_state != NULL ) {
          // graph search
        }
        // graph search
        // and then run the comprehensive prediction to update the next sensor to hit
        break;
      default:
        assertm( 1, false, "ERROR: unrecognized request: %d", receive_msg.request_type );
        break;
    }
  }
}
