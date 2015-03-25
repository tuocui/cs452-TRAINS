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
  assertu( 1, rail_server_tid > 0 );
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
    ret_val = Send( rail_server_tid, (char*)&rail_msg, sizeof( rail_msg ), (char*)&train_state, sizeof( train_state ));
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
    assertu( 1, train_state );

    init_rail_cmds( &rail_cmds );
    request_next_command( train_state, &rail_cmds );
  }
}

void sensor_data_courier( ) {
  int rail_server_tid = MyParentTid( );
  assertu( 1, rail_server_tid > 0 );
  
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
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
    (rail_msg.to_server_content.sensor_data)->sensor_num = sensor_num;
    ret_val = Send( rail_server_tid, (char*)&rail_msg, sizeof( rail_msg ), (char *)&rail_msg, 0);
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
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
  int i;
  train_state_t *trains;
  sensor_args_t sensor_args;
  ret_val = Receive( &rail_server_tid, (char *)&rail_msg, 0 );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  ret_val = Reply( rail_server_tid, (char *)&rail_msg, 0 );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  FOREVER {
    ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&sensor_args, sizeof(sensor_args) );
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
    trains = sensor_args.trains;
    sensor_num = sensor_args.sensor_num;
    
    // TODO: Change this function to also include fallback sensors
    expected_train_idx = get_expected_train_idx( trains, sensor_num );
    train_state_t *train = expected_train_idx == NONE ? NULL : &( trains[expected_train_idx] ); 
    //assertu( 1, train );
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
      train->init_time = cur_time;
      train->state = READY;
      Printf( COM2, "\0337\033[1A\033[2K\rTrain %d INITIALIZED\0338", train->train_id );
      Printf( COM2, "\0337\033[21;%dHExpected distance to N/A: N/A    \0338", get_train_idx( train->train_id ) * 37 );
      Printf( COM2, "\0337\033[20;%dHActual distance to N/A: N/A    \0338", get_train_idx( train->train_id ) * 37 );
      Printf( COM2, "\0337\033[19;%dHDistance difference: N/A    \0338", get_train_idx( train->train_id ) * 37 );
      clear_reservations_by_train( train->track_graph, train );
    } else {
      // Do not update velocity if we have picked up the the train from a fallback sensor
      if( train->next_sensor_id == sensor_num ) {
        update_velocity( train, cur_time, train->time_at_last_landmark, train->dist_to_next_sensor );
        sensor_id_to_name( train->next_sensor_id, sensor_name );
        Printf( COM2, "\0337\033[21;%dHExpected distance to %c%c%c: %d    \0338", get_train_idx( train->train_id ) * 37, sensor_name[0], sensor_name[1], sensor_name[2], train->mm_past_landmark / 10 );
        Printf( COM2, "\0337\033[20;%dHActual distance to %c%c%c: %d    \0338", get_train_idx( train->train_id ) * 37, sensor_name[0], sensor_name[1], sensor_name[2], train->dist_to_next_sensor );
        Printf( COM2, "\0337\033[19;%dHDistance difference: %d    \0338", get_train_idx( train->train_id ) * 37, ( train->mm_past_landmark / 10 ) - train->dist_to_next_sensor );
        //clear_prev_train_reservation( train );
      } else {
        Printf( COM2, "\0337\033[12;1HWOAH NELLY, ALMOST LOST TRAIN %d at time: %d\0338", train->train_id, cur_time );
        clear_reservations_by_train( train->track_graph, train );
      }
      train->vel_at_last_landmark = train->cur_vel;
    }
    //assertu( 2, train->next_sensor_id == sensor_num );
    rail_msg.to_server_content.train_state = train;
    // updates next_sensor_id and dist_to_next_sensor
    train->time_at_last_landmark = cur_time;
    debugu( 4, "updating prev_sensor_id to: %d", sensor_num );
    train->prev_sensor_id = sensor_num;
    train->mm_past_landmark = 0;
    if( train->state == REVERSING ) {
      predict_next_sensor_dynamic( train );
    } else {
      predict_next_sensor_static( train );
    }
    predict_next_fallback_sensors_static( train );
    for( i = 0; i < NUM_FALLBACK && (train->fallback_sensors)[i] != -1; ++i ) {
      (train->time_to_fallback_sensor)[i] = time_to_node( train, (train->fallback_dist)[i], cur_time ) + ( cur_time * 10 );
    }
    sensor_id_to_name( train->next_sensor_id, sensor_name );
    Printf( COM2, "\0337\033[17;%dHNext expected sensor: %c%c%c    \0338", get_train_idx( train->train_id ) * 37, sensor_name[0], sensor_name[1], sensor_name[2] );
    // if no reverse, 
  }
}

void train_exe_worker( ) {
  rail_msg_t rail_msg;
  rail_msg.request_type = TRAIN_EXE_READY;
  rail_msg.to_server_content.nullptr = NULL;
  rail_msg.from_server_content.nullptr = NULL;
  rail_msg.general_val = NONE;
  int ret_val;
  int rail_server_tid;
  train_state_t *train;
  char sensor_name[4];
  train_cmd_args_t train_cmd_args;
  ret_val = Receive( &rail_server_tid, (char *)&train, sizeof( train ) );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  ret_val = Reply( rail_server_tid, (char *)&rail_msg, 0 );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );

  FOREVER {
    ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&train_cmd_args, sizeof(train_cmd_args) );
    rail_msg.general_val = NONE;
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
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
        debugu( 4, "train_exe_worker received reverse request, reversing now ... " );
        train->state = REVERSING;
        int prev_speed = train->cur_speed;
        int stopping_time = get_cur_stopping_time( train ) / 10;
        set_train_speed( train, 0 );
        if( train->state != NOT_INITIALIZED && train->state != INITIALIZING ) {
          predict_next_sensor_dynamic( train );
          predict_next_fallback_sensors_static( train );
          sensor_id_to_name( train->next_sensor_id, sensor_name );
          // TODO FIXME: MAKE THIS NOT BE IN THE TRAIN EXE WORKER
          int stopping_dist = get_cur_stopping_distance( train ) + STOP_BUFFER + get_len_train_behind( train ) + ( train->mm_past_landmark / 10 );
          track_node_t *cur_node = &(train->track_graph[train->prev_sensor_id]);
          track_node_t *next_node;
          int branch_ind;
          while( stopping_dist >= 0 ) {
            if( cur_node->type == NODE_BRANCH ) {
              branch_ind = cur_node->num;
              if( branch_ind > 152 ) {
                branch_ind -= 134;
              }
              next_node = cur_node->edge[train->switch_states[branch_ind]].dest;
              stopping_dist -= cur_node->edge[train->switch_states[branch_ind]].dist;
            } else {
              next_node = cur_node->edge[DIR_AHEAD].dest;
              stopping_dist -= cur_node->edge[DIR_AHEAD].dist;
            }
            if( next_node->type == NODE_MERGE && next_node->reverse->num != train->rev_branch_ignore ) {
              if( next_node->reverse->edge[DIR_STRAIGHT].dest == cur_node->reverse ) {
                Printf( COM2, "Switching %d to %d \r\n", next_node->reverse->num, DIR_STRAIGHT );
                assertu( 1, next_node->reverse->type == NODE_BRANCH );
                set_switch( next_node->reverse->num, STRAIGHT, train->switch_states );
              } else {
                Printf( COM2, "Switching %d to %d \r\n", next_node->reverse->num, DIR_CURVED );
                assertu( 1, next_node->reverse->type == NODE_BRANCH );
                set_switch( next_node->reverse->num, CURVED, train->switch_states );
              }
            }
            cur_node = next_node;
          }
          Printf( COM2, "\0337\033[17;%dHNext expected sensor: %c%c%c    \0338", get_train_idx( train->train_id ) * 37, sensor_name[0], sensor_name[1], sensor_name[2] );
        }
        Delay( stopping_time + STOP_TIME_BUFFER ); // TODO: Change this to stopping time;
        set_train_speed( train, 15 );
        set_train_speed( train, prev_speed );
        train->state = READY;
        rail_msg.general_val = train->train_id;
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
        train->prev_dest_id = train->dest_id;
        train->dest_id = train_cmd_args.dest;
        train->mm_past_dest = train_cmd_args.mm_past_dest;
        char dest[3];
        sensor_id_to_name( train->dest_id, dest );
        if( train->dest_id == NONE ) {
          Printf( COM2, "\0337\033[14;%dHCurrent destination: N/A\0338", get_train_idx( train->train_id ) * 37 );
          //Printf( COM2, "%d Current destination: N/A", get_train_idx( train->train_id ) * 37 );
        } else {
          Printf( COM2, "\0337\033[14;%dHCurrent destination: %c%c%c\0338", get_train_idx( train->train_id ) * 37, dest[0], dest[1], dest[2] );
          //Printf( COM2, "%d Current destination: %c%c%c", get_train_idx( train->train_id ) * 37, dest[0], dest[1], dest[2] );
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
  track_node_t *graph;
  switch_cmd_args_t switch_cmd_args;
  ret_val = Receive( &rail_server_tid, (char *)&switch_num, sizeof( switch_num ) );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  ret_val = Reply( rail_server_tid, (char *)&rail_msg, 0 );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );

  FOREVER {
    ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&switch_cmd_args, sizeof(switch_cmd_args) );
    debugu( 4, "SWITCH_EXE_WORKER AFTER SEND: %d received request to change to: %d ", switch_num, switch_cmd_args.state );
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
    if( switch_cmd_args.delay_time > 0 ) {
      ret_val = Delay( switch_cmd_args.delay_time );
      assertum( 1, ret_val >= 0, "retval: %d", ret_val );
    }
    state = switch_cmd_args.state;
    switch_states = switch_cmd_args.switch_states;
    graph = switch_cmd_args.graph;
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
  int i;
  int cur_time;
  char dest[4];
  //bool all_stopped;
  rail_cmds_t rail_cmds;
  rail_msg_t rail_msg;
  rail_msg.request_type = COLLISION_CMDS;
  rail_msg.to_server_content.rail_cmds = &rail_cmds;
  rail_msg.from_server_content.nullptr = NULL;
  update_train_args_t update_train_args;
  ret_val = Receive( &rail_server_tid, (char *)&update_train_args, sizeof( update_train_args ) );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  ret_val = Reply( rail_server_tid, (char *)&trains, 0 );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  trains = update_train_args.trains;
  FOREVER {
    cur_time = Delay( 1 );
    //all_stopped = 1;
    for( i = 0; i < TR_MAX; ++i ) {
      ((rail_msg.to_server_content).rail_cmds)->train_id = NONE;
      ((rail_msg.to_server_content).rail_cmds)->train_action = NONE;
      ((rail_msg.to_server_content).rail_cmds)->train_speed = -1;
      ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
      if( trains[i].state != NOT_INITIALIZED && trains[i].state != INITIALIZING ) {
        trains[i].cur_vel = get_cur_velocity( &(trains[i]), cur_time );
        trains[i].mm_past_landmark = get_mm_past_last_landmark( &(trains[i]), cur_time );
        trains[i].time_since_last_pos_update = cur_time;
        trains[i].vel_at_last_pos_update = trains[i].cur_vel;
        trains[i].time_to_next_sensor = time_to_node( &(trains[i]), trains[i].dist_to_next_sensor, cur_time );
        // If we haven't hit a landmark in 15s, reinitialize the train
        if( cur_time - trains[i].time_at_last_landmark > 2500 && 
            trains[i].cur_speed != 0 && 
            cur_time - trains[i].init_time > 2500 ) {
          ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
          ((rail_msg.to_server_content).rail_cmds)->train_action = TR_INIT;
          ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
          Printf( COM2, "\0337\033[1A\033[2K\rUh oh... we lost Train %d. Reinitializing.\0338", trains[i].train_id );
        }
        if( trains[i].cur_speed == 0 &&
            cur_time - trains[i].speed_change_time > 2500 ) {
          ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
          ((rail_msg.to_server_content).rail_cmds)->train_action = TR_CHANGE_SPEED;
          ((rail_msg.to_server_content).rail_cmds)->train_speed = 10;
          ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
          ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
        }
        /* clear destination if stopping flag is set */
        // TODO: Use cur_vel to know whe nthe train stops
        if( trains[i].train_reach_destination && trains[i].cur_vel == 0 ) {
          trains[i].train_reach_destination = false;
          trains[i].prev_dest_id = trains[i].dest_id;
          trains[i].dest_id = get_rand_dest( cur_time, trains[i].track_graph );
          sensor_id_to_name( trains[i].dest_id, dest );
          ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
          ((rail_msg.to_server_content).rail_cmds)->train_action = TR_CHANGE_SPEED;
          ((rail_msg.to_server_content).rail_cmds)->train_speed = 10;
          ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
          ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
          Printf( COM2, "\0337\033[14;%dHCurrent destination: %c%c%c\0338", i * 37, dest[0], dest[1], dest[2] );
        }
      }
    }
    i = cur_time % TR_MAX;
    if( trains[i].state != NOT_INITIALIZED && trains[i].state != INITIALIZING ) {
      ret_val = update_track_reservation( &(trains[i]), trains );
      if( ret_val == -1 ) {
        ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
        ((rail_msg.to_server_content).rail_cmds)->train_action = TR_REVERSE;
        ((rail_msg.to_server_content).rail_cmds)->train_speed = -1;
        ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
        ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
      } else if( ret_val == -2 ) {
        ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
        ((rail_msg.to_server_content).rail_cmds)->train_action = TR_CHANGE_SPEED;
        ((rail_msg.to_server_content).rail_cmds)->train_speed = 8;
        ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
        ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
      } else if( ret_val == -3 ) {
        ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
        ((rail_msg.to_server_content).rail_cmds)->train_action = TR_STOP;
        ((rail_msg.to_server_content).rail_cmds)->train_speed = 0;
        ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
        ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
      } else if( ret_val > 0 ) {
        ((rail_msg.to_server_content).rail_cmds)->train_id = ret_val;
        ((rail_msg.to_server_content).rail_cmds)->train_action = TR_CHANGE_SPEED;
        ((rail_msg.to_server_content).rail_cmds)->train_speed = 12;
        ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
        ret_val = Send( rail_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
      }
    }
  }
}

void print_trains( ) {
  train_state_t *trains;
  int ret_val;
  int rail_server_tid;
  int i;
  int cur_time;
  update_train_args_t update_train_args;
  ret_val = Receive( &rail_server_tid, (char *)&update_train_args, sizeof( update_train_args ) );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  ret_val = Reply( rail_server_tid, (char *)&trains, 0 );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  trains = update_train_args.trains;
  FOREVER {
    cur_time = Delay( 10 );
    for( i = 0; i < TR_MAX; ++i ) {
      if( trains[i].state != NOT_INITIALIZED && trains[i].state != INITIALIZING ) {
        Printf( COM2, "\0337\033[15;%dH%d    \0338", ( i * 37 ) + 8, trains[i].cur_speed );
        Printf( COM2, "\0337\033[18;%dH%d    \0338", ( i * 37 ) + 10, trains[i].mm_past_landmark / 10 );
        Printf( COM2, "\0337\033[16;%dH%d    \0338", ( i * 37 ) + 10, trains[i].cur_vel );
        print_rsv( &(trains[i]), trains );
      } else if( trains[i].state == INITIALIZING ) {
        Printf( COM2, "\0337\033[13;%dH===== TRAIN %d ====\0338", ( i * 37 ), trains[i].train_id );
        Printf( COM2, "\0337\033[15;%dHSpeed: N/A\0338", ( i * 37 ) );
        Printf( COM2, "\0337\033[18;%dHmm past: N/A\0338", ( i * 37 ) );
        Printf( COM2, "\0337\033[16;%dHCur vel: N/A\0338", ( i * 37 ) );
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
  //init_tracka( (track_node_t*)track_graph );
  init_trackb( (track_node_t*)track_graph );
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
    rail_graph_worker_tids[i] = Create( 11, &rail_graph_worker );
    assertu( 1, rail_graph_worker_tids[i] > 0 );
  }
  rail_cmds_t* recved_cmds; 

  int update_trains_tid = Create( 13, &update_trains );
  update_train_args_t update_train_args;
  update_train_args.trains = trains;
  ret_val = Send( update_trains_tid, (char *)&update_train_args, sizeof( update_train_args ), (char *)&client_tid, 0 );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );

  int print_trains_tid = Create( 14, &print_trains );
  ret_val = Send( print_trains_tid, (char *)&update_train_args, sizeof( update_train_args ), (char *)&client_tid, 0 );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );


  int worker_tid;
  /* Sensor worker declarations */
  int sensor_courier_tid = Create( 2, &sensor_data_courier );
  assertu( 1, sensor_courier_tid > 0 );

  declare_ring_queue( int, sensor_workers, SENSOR_WORKER_MAX );
  int sensor_worker_tids[SENSOR_WORKER_MAX];
  char sensor_name[4];
  for( i = 0; i < SENSOR_WORKER_MAX; ++i ) {
    sensor_worker_tids[i] = Create( 10, &sensor_worker );
    assertu( 1, sensor_worker_tids[i] > 0 );
    ret_val = Send( sensor_worker_tids[i], (char *)&client_tid, 0, (char *)&client_tid, 0 );
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  }
  sensor_args_t sensor_args;
  sensor_args.trains = trains;

  /* Train action workers */
  int train_exe_worker_tids[TR_MAX];
  int train_exe_worker_tid = NONE;
  for( i = 0; i < TR_MAX; ++i ) {
    train_exe_worker_tids[i] = Create( 10, &train_exe_worker );
    assertu( 1, train_exe_worker_tids[i] > 0 );
    train_state_t *train = &(trains[i]);
    ret_val = Send( train_exe_worker_tids[i], (char *)&train, sizeof( train ), (char *)&client_tid, 0 );
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  }
  train_cmd_args_t train_cmd_args;

  /* Switch action workers */
  int switch_exe_worker_tid = NONE;
  int switch_exe_worker_tids[SW_MAX];
  int switch_num;
  for( i = 1; i < SW_MAX; ++i ) {
    switch_exe_worker_tids[i] = Create( 10, &switch_exe_worker );
    assertu( 1, switch_exe_worker_tids[i] > 0 );
    switch_num = i;
    if( switch_num > 18 ) {
      switch_num += 134;
    }
    ret_val = Send( switch_exe_worker_tids[i], (char *)&switch_num, sizeof( switch_num ), (char *)&client_tid, 0 );
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  }
  switch_cmd_args_t switch_cmd_args;
  switch_cmd_args.switch_states = switch_states;
  switch_cmd_args.graph = track_graph;

  FOREVER { 
    ret_val = Receive( &client_tid, (char *)&receive_msg, sizeof( rail_msg_t ));
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
    switch( receive_msg.request_type ) {
      case SENSOR_DATA:
        {
        //DEBUG
          /* retrieve data, find sensor number and corresponding train, set ready */
          ret_val = Reply( client_tid, (char *)&receive_msg, 0 );
          assertum( 1, ret_val >= 0, "retval: %d", ret_val );
          if( !sensor_workers_empty( ) ) {
            int sensor_num = (receive_msg.to_server_content.sensor_data)->sensor_num;
            worker_tid = sensor_workers_pop_front( );
            sensor_args.sensor_num = sensor_num;
            ret_val = Reply( worker_tid, (char *)&sensor_args, sizeof(sensor_args) );
            assertum( 1, ret_val >= 0, "retval: %d", ret_val );
          }
        }
        break;
      case COLLISION_CMDS:
        // just fall through to user input, same diff
      case USER_INPUT:
        // TODO: If user/collision reverse, make sure we properly set switches ahead so we don't derail
        ret_val = Reply( client_tid, (char *)&receive_msg, 0 );
        assertum( 1, ret_val >= 0, "retval: %d", ret_val );
        if( (receive_msg.to_server_content.rail_cmds)->train_id != NONE ) {
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
            assertum( 1, ret_val == 0, "ret_val: %d", ret_val );
          }
        } else if ( receive_msg.to_server_content.rail_cmds->switch_idx != NONE ) {
          assertum( 1, receive_msg.to_server_content.rail_cmds->switch_idx == 0, 
              "user packed %d sw_cmds, should it?", receive_msg.to_server_content.rail_cmds->switch_idx + 1 );
          int switch_id = receive_msg.to_server_content.rail_cmds->switch_cmds[0].switch_id;
          switch_exe_worker_tid = switch_exe_worker_tids[switch_id];
          switch_cmd_args.state = receive_msg.to_server_content.rail_cmds->switch_cmds[0].switch_action;
          switch_cmd_args.delay_time = 0;
          ret_val = Reply( switch_exe_worker_tid, (char *)&switch_cmd_args, sizeof( switch_cmd_args ) );
          assertum( 1, ret_val >= 0, "retval: %d", ret_val );
          int other_switch_id = NONE;
          if( receive_msg.to_server_content.rail_cmds->switch_cmds[0].switch_action == SW_CURVED ) {
            switch( switch_id ) {
            case SW153:
              other_switch_id = SW154;
              break;
            case SW154:
              other_switch_id = SW153;
              break;
            case SW155:
              other_switch_id = SW156;
              break;
            case SW156:
              other_switch_id = SW155;
              break;
            }
          }
          if( other_switch_id != NONE ) {
            switch_exe_worker_tid = switch_exe_worker_tids[other_switch_id];
            switch_cmd_args.state = SW_STRAIGHT;
            switch_cmd_args.delay_time = 0;
            ret_val = Reply( switch_exe_worker_tid, (char *)&switch_cmd_args, sizeof( switch_cmd_args ) );
            assertum( 1, ret_val >= 0, "retval: %d", ret_val );
          }
        } else if ( receive_msg.to_server_content.rail_cmds->rsv_node_id != NONE ) {
          int rsv_node_id = receive_msg.to_server_content.rail_cmds->rsv_node_id;
          int rsv_node_dir = receive_msg.to_server_content.rail_cmds->rsv_node_dir;
          track_edge_t *edge = &(track_graph[rsv_node_id].edge[rsv_node_dir]);
          if( edge->middle_train_num == USER_INPUT_NUM || edge->begin_train_num == USER_INPUT_NUM ) {
            edge->middle_train_num = NONE;
            edge->begin_train_num = NONE;
            Printf( COM2, "\0337\033[1A\033[2K\rUnreserved track from %s, direction %d\0338", track_graph[rsv_node_id].name, rsv_node_dir );
          } else if( edge->middle_train_num == NONE && edge->begin_train_num == NONE ) {
            edge->middle_train_num = USER_INPUT_NUM;
            edge->begin_train_num = USER_INPUT_NUM;
            Printf( COM2, "\0337\033[1A\033[2K\rReserved track from %s, direction %d\0338", track_graph[rsv_node_id].name, rsv_node_dir );
          } else {
            Printf( COM2, "\0337\033[1A\033[2K\rCould not reserve track from %s, direction %d\0338", track_graph[rsv_node_id].name, rsv_node_dir );
          }
        }
        break;
      case RAIL_CMDS:
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
        if( train_exe_worker_tid != NONE ) {
          debugu( 4, "server got requested train_exe_worker_tid: %d", train_exe_worker_tid );
          train_cmd_args.cmd = recved_cmds->train_action;
          train_cmd_args.speed_num = recved_cmds->train_speed;
          train_cmd_args.delay_time = recved_cmds->train_delay;
          ret_val = Reply( train_exe_worker_tid, (char*)&train_cmd_args, sizeof( train_cmd_args ));
          assertum( 1, ret_val >= 0, "retval: %d", ret_val ); //asdasdasdasd
        }
        /* get and send switch cmds */
        for( i = 0; i <= recved_cmds->switch_idx; ++i ) {
          debugu( 4, "before reply to switch_exe_worker: switch_idx: %d, i: %d", recved_cmds->switch_idx, i );
          switch_exe_worker_tid = switch_exe_worker_tids[recved_cmds->switch_cmds[i].switch_id];
          switch_cmd_args.state = recved_cmds->switch_cmds[i].switch_action;
          switch_cmd_args.delay_time = recved_cmds->switch_cmds[i].switch_delay;
          ret_val = Reply( switch_exe_worker_tid, (char*)&switch_cmd_args, sizeof( switch_cmd_args ));
          debugu( 4, "after reply to switch_exe_worker" );
          assertum( 1, ret_val >= 0, "retval: %d, switch_worker_tid: %d, switch_id: %d", ret_val, 
              switch_exe_worker_tid, recved_cmds->switch_cmds[i].switch_id );
          int other_switch_id = NONE;
          if( recved_cmds->switch_cmds[i].switch_action == SW_CURVED ) {
            switch( recved_cmds->switch_cmds[i].switch_id ) {
            case SW153:
              other_switch_id = SW154;
              break;
            case SW154:
              other_switch_id = SW153;
              break;
            case SW155:
              other_switch_id = SW156;
              break;
            case SW156:
              other_switch_id = SW155;
              break;
            }
          }
          if( other_switch_id != NONE ) {
            switch_exe_worker_tid = switch_exe_worker_tids[other_switch_id];
            switch_cmd_args.state = SW_STRAIGHT;
            switch_cmd_args.delay_time = 0;
            ret_val = Reply( switch_exe_worker_tid, (char *)&switch_cmd_args, sizeof( switch_cmd_args ) );
            assertum( 1, ret_val >= 0, "retval: %d", ret_val ); //sdfsdfsdf
          }
        }
        break;
      case TRAIN_EXE_READY:
        {
          if( receive_msg.general_val != NONE ) {
            train_state_t *train = &(trains[get_train_idx( receive_msg.general_val )]);
            if( train->state != NOT_INITIALIZED && train->state != INITIALIZING ) {
              update_prev_sensor_id_for_rev( train );
              train->rev_branch_ignore = NONE;
            }
          }
        }
        //DEBUG
        // rerun graph search only if command was given by user AND if command is reverse or change speed
        // Don't run graph search if train state is busy, or reversing
        // Update train direction if reverse is given
        break;
      case SWITCH_EXE_READY:
        //DEBUG
        for( i = 0; i < TR_MAX; ++i ) {
          if( trains[i].state != NOT_INITIALIZED && trains[i].state != INITIALIZING ) {
            if( trains[i].state == REVERSING ) {
              predict_next_sensor_dynamic( &(trains[i]) );
            } else {
              predict_next_sensor_static( &(trains[i]) );
            }
            predict_next_fallback_sensors_static( &(trains[i]) );
            sensor_id_to_name( trains[i].next_sensor_id, sensor_name );
            Printf( COM2, "\0337\033[17;%dHNext expected sensor: %c%c%c    \0338", i * 37, sensor_name[0], sensor_name[1], sensor_name[2] );
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
              assertum( 1, ret_val == 0, "ret_val: %d, tid: %d", ret_val, rail_graph_worker_tids[i] );
            }
          }
        }
        //TODO: run dynamic graph search
        break;
      default:
        assertum( 1, false, "ERROR: unrecognized request: %d", receive_msg.request_type );
        break;
    }
  }
}
