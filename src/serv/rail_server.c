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
  int dist_to_next_sensor;
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
    train_state_t *train = &(trains[TRAIN_58]); 
    cur_time = Time( );
    if( train->state == INITIALIZING ) {
      set_train_speed( train->train_id, 0 );
      train->cur_speed = 0;
      train->prev_speed = 0;
      train->speed_change_time = cur_time;
    } else {
      update_velocity( train, cur_time, train->time_at_last_landmark, train->dist_to_next_sensor );
    }
    //assert( 2, train->next_sensor_id == sensor_num );
    train->prev_sensor_id = sensor_num;
    train->next_sensor_id = get_next_sensor(train, &dist_to_next_sensor); // TODO: WILSON, in rail_helper.c
    train->time_at_last_landmark = cur_time;
    train->mm_past_landmark = 0;
    train->dist_to_next_sensor = dist_to_next_sensor;
    rail_msg.to_server_content.train_state = train;
    // run prediction
  }
}

void rail_server( ) {
  if( RegisterAs( (char*)RAIL_SERVER ) == -1 ) {
    bwputstr( COM2, "ERROR: failed to register rail_server, aborting ...\n\r" );
    Exit( );
  }
  int client_tid;
  
  //int switch_states[SW_MAX];
  int train_graph_search_tid[TR_MAX]; 
  //bool train_graph_search_ready[TR_MAX];
  //bool train_delay_treads_arrived[TR_MAX]; TODO
  train_state_t trains[TR_MAX];
  rail_msg_t receive_msg;

  init_trains( trains, TR_MAX );

  int i;
  for( i = 0; i < TR_MAX; ++i ) {
    train_graph_search_tid[i] = Create( 10, &rail_graph_worker );
    assert( 1, train_graph_search_tid[i] > 0 );
  }
  
  int sensor_courier_tid = Create( 2, &sensor_data_courier );
  assert( 1, sensor_courier_tid > 0 );

  //TODO: add secretary/courier

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
  /*
  int train_conductor_tids[TR_MAX];
  for( i = 0; i < TR_MAX; ++i ) {
    train_worker_tids[i] = Create( 10, &sensor_worker );
    assert( 1, sensor_worker_tids[i] > 0 );
    Send( sensor_worker_tids[i], (char *)&client_tid, 0, (char *)&client_tid, 0 );
  }*/
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
        if( (receive_msg.to_server_content.rail_cmds)->train_id ) {
          
        } else if ( (receive_msg.to_server_content.rail_cmds)->switch_id0 ) {

        }
        break;
      case TIMER_READY:
        // update all trains positions
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
          Printf( COM2, "just got back stuff for train: %d\r\n", (receive_msg.to_server_content.train_state)->train_id );
        }
        // graph search
        break;
      default:
        assertm( 1, false, "ERROR: unrecognized request: %d", receive_msg.request_type );
        break;
    }
  }
}
