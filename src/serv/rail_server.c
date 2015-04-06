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
#include "screen.h"

#include "track_data_new.h"
#include "track_node.h"

inline void print_stop_sign( ) {
  Printf( COM2, "\0337\033[28;0H\033[1;31m\
       u ************************** u\n\r\
     u                                u\n\r\
   u                                    u\n\r\
 u                                        u\n\r\
*                           _              *\n\r\
* __      ____ _ _ __ _ __ (_)_ __   __ _  *\n\r\
* \\ \\ /\\ / / _` | '__| '_ \\| | '_ \\ / _` | *\n\r\
*  \\ V  V / (_| | |  | | | | | | | | (_| | *\n\r\
*   \\_/\\_/ \\__,_|_|  |_| |_|_|_| |_|\\__, | *\n\r\
*                                   |___/  *\n\r\
 u                                        u\n\r\
   u                                    u\n\r\
     u                                u\n\r\
       u............................u\0338\033[0m" );
}

inline void erase_stop_sign( ) {
Printf( COM2, "\0337\033[28;0H\
                                        \n\r\
                                         \n\r\
                                           \n\r\
                                             \n\r\
                                               \n\r\
                                               \n\r\
                                               \n\r\
                                                    \n\r\
                                                \n\r\
                                                   \n\r\
                                               \n\r\
                                              \n\r\
                                            \n\r\
                                          \n\r\
                                          \0338" );

}

void rail_graph_worker( ) {
  //DEBUG
  int cmd_server_tid = WhoIs(( char* )CMD_SERVER );
  assertu( 1, cmd_server_tid > 0 );
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
    /* send to cmd_server, but reply msg is from rail_server */
    ret_val = Send( cmd_server_tid, (char*)&rail_msg, sizeof( rail_msg ), (char*)&train_state, sizeof( train_state ));
    debugu( 1, "graph worker just sent last command to cmd_server" );
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
    ret_val = Send( sensor_task_id, (char *)&sensor_num, 0, (char *)&sensor_num, sizeof(sensor_num) );
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
      case TRAIN_62_NUM:
        init_62( train );
        break;
      case TRAIN_24_NUM:
        init_24( train );
        break;
      case TRAIN_45_NUM:
        init_45( train );
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
      Printf( COM2, "\0337\033[14;%dHCurrent destination: N/A\0338", get_train_idx( train->train_id ) * 37 );
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
  int cmd_server_tid;
  train_state_t *train;
  char sensor_name[4];
  train_cmd_args_t train_cmd_args;
  ret_val = Receive( &cmd_server_tid, (char *)&train, sizeof( train ) );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  ret_val = Reply( cmd_server_tid, (char *)&rail_msg, 0 );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  rail_msg.train_or_switch_id = train->train_id;

  FOREVER {
    ret_val = Send( cmd_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&train_cmd_args, sizeof(train_cmd_args) );
    rail_msg.general_val = NONE;
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
    // TODO: Make this better, serious race condition issue right now
    if( train_cmd_args.delay_time > 0 ) {
      train->state = BUSY;
      Delay( train_cmd_args.delay_time );
      train->state = READY;
    }
    switch( train_cmd_args.cmd ) {
    case TR_STOP:
      if( train->state != INITIALIZING && train->state != NOT_INITIALIZED ) {
        train->state = STOPPING;
      }
      set_train_speed( train, 0 );
      if( train->train_reach_destination ) {
        train->dest_id = NONE;
        train->train_reach_destination = false;
        Printf( COM2, "\0337\033[14;%dHCurrent destination: N/A\0338", get_train_idx( train->train_id ) * 37 );
      }
      if( train->state != INITIALIZING && train->state != NOT_INITIALIZED ) {
        train->state = READY;
      }
      break;
    case TR_REVERSE:
      {
        debugu( 4, "train_exe_worker received reverse request, reversing now ... " );
        if( train->state != INITIALIZING && train->state != NOT_INITIALIZED ) {
          train->state = REVERSING;
        }
        int prev_speed = train->cur_speed;
        set_train_speed( train, 0 );
        int stopping_time = get_cur_stopping_time( train ) / 10;
        if( train->state != NOT_INITIALIZED && train->state != INITIALIZING ) {
          predict_next_sensor_dynamic( train );
          predict_next_fallback_sensors_static( train );
          sensor_id_to_name( train->next_sensor_id, sensor_name );
          // TODO FIXME: MAKE THIS NOT BE IN THE TRAIN EXE WORKER
          int stopping_dist = get_cur_stopping_distance( train ) + ( ( 5 * STOP_BUFFER ) / 4 ) + get_len_train_behind( train ) + ( train->mm_past_landmark / 10 );
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
            } else if( cur_node->type == NODE_EXIT ) {
              break;
            } else {
              next_node = cur_node->edge[DIR_AHEAD].dest;
              stopping_dist -= cur_node->edge[DIR_AHEAD].dist;
            }
            if( next_node->type == NODE_MERGE && next_node->reverse->num != train->rev_branch_ignore ) {
              if( next_node->reverse->edge[DIR_STRAIGHT].dest == cur_node->reverse ) {
                assertu( 1, next_node->reverse->type == NODE_BRANCH );
                set_switch( next_node->reverse->num, STRAIGHT, train->switch_states );
              } else {
                assertu( 1, next_node->reverse->type == NODE_BRANCH );
                set_switch( next_node->reverse->num, CURVED, train->switch_states );
              }
            }
            cur_node = next_node;
          }
          Printf( COM2, "\0337\033[17;%dHNext expected sensor: %c%c%c    \0338", get_train_idx( train->train_id ) * 37, sensor_name[0], sensor_name[1], sensor_name[2] );
        }
        Delay( stopping_time );
        set_train_speed( train, 15 );
        set_train_speed( train, prev_speed );
        if( train->state != INITIALIZING && train->state != NOT_INITIALIZED ) {
          train->state = READY;
          rail_msg.general_val = train->train_id;
        }
        break;
      }
    case TR_CHANGE_SPEED:
      if( train->state != INITIALIZING && train->state != NOT_INITIALIZED ) {
        train->state = BUSY;
      }
      set_train_speed( train, train_cmd_args.speed_num );
      if( train->state != INITIALIZING && train->state != NOT_INITIALIZED ) {
        train->state = READY;
      }
      break;
    case TR_INIT:
      train->state = INITIALIZING;
      set_train_speed( train, INIT_SPEED );
      break;
    case TR_DEST:
      {
        if( train->dest_id != NONE ) {
          train->prev_dest_id = train->dest_id;
        }
        train->dest_id = train_cmd_args.dest;
        train->mm_past_dest = train_cmd_args.mm_past_dest;
        char dest[3];
        sensor_id_to_name( train->dest_id, dest );
        if( train->dest_id == NONE ) {
          Printf( COM2, "\0337\033[14;%dHCurrent destination: N/A\0338", get_train_idx( train->train_id ) * 37 );
        } else {
          Printf( COM2, "\0337\033[14;%dHCurrent destination: %c%c%c\0338", get_train_idx( train->train_id ) * 37, dest[0], dest[1], dest[2] );
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
    case TR_RAND_DEST:
      train->set_rand_dest ^= 1;
      if( train->set_rand_dest ) {
        Printf( COM2, "\0337\033[1A\033[2K\rTrain %d now going to random destinations\0338", train->train_id );
      } else {
        Printf( COM2, "\0337\033[1A\033[2K\rTrain %d now obeying orders\0338", train->train_id );
      }
      break;
    case TR_SPEED_UP:
      {
        if( train->state != INITIALIZING && train->state != NOT_INITIALIZED ) {
          train->state = BUSY;
        }
        int cur_speed_normalized = train->cur_speed > 14 ? train->cur_speed - 15 : train->cur_speed;
        if( cur_speed_normalized == 10 ) {
          if( train->state != INITIALIZING && train->state != NOT_INITIALIZED ) {
            train->state = READY;
          }
          break;
        }
        if( cur_speed_normalized == 0 ) {
          cur_speed_normalized += 7;
        }
        set_train_speed( train, cur_speed_normalized + 1 );
        if( train->state != INITIALIZING && train->state != NOT_INITIALIZED ) {
          train->state = READY;
        }
        break;
      }
    case TR_SLOW_DOWN:
      {
        if( train->state != INITIALIZING && train->state != NOT_INITIALIZED ) {
          train->state = BUSY;
        }
        int cur_speed_normalized = train->cur_speed > 14 ? train->cur_speed - 15 : train->cur_speed;
        if( cur_speed_normalized == 0 ) {
          if( train->state != INITIALIZING && train->state != NOT_INITIALIZED ) {
            train->state = READY;
          }
          break;
        }
        if( cur_speed_normalized == 8 ) {
          cur_speed_normalized -= 7;
        }
        set_train_speed( train, cur_speed_normalized - 1 );
        if( train->state != INITIALIZING && train->state != NOT_INITIALIZED ) {
          train->state = READY;
        }
        break;
      }
    case TR_SWITCH_RIGHT:
      {
        int branch_ind = train->next_branch_num > 152 ? ( ( train->next_branch_num - 134 ) * 2 ) + 78 : 
                                                        ( train->next_branch_num * 2) + 78;
        assert( 1, train->track_graph[branch_ind].type == NODE_BRANCH );
        int switch_dir = train->track_graph[branch_ind].switch_dir;
        if( switch_dir == DIR_LEFT ) {
          set_switch( train->next_branch_num, STRAIGHT, train->switch_states );
          int other_switch_num = NONE;
          switch( train->next_branch_num ) {
          case 154:
            other_switch_num = 153;
            break;
          case 156:
            other_switch_num = 155;
            break;
          }
          if( other_switch_num != NONE ) {
            set_switch( other_switch_num, CURVED, train->switch_states );
          }
        } else if( switch_dir == DIR_RIGHT ) {
          set_switch( train->next_branch_num, CURVED, train->switch_states );
        }
        rail_msg.general_val = UPDATE_POS;
        break;
      }
    case TR_SWITCH_LEFT:
      {
        int branch_ind = train->next_branch_num > 152 ? ( ( train->next_branch_num - 134 ) * 2 ) + 78 : 
                                                        ( train->next_branch_num * 2) + 78;
        assert( 1, train->track_graph[branch_ind].type == NODE_BRANCH );
        int switch_dir = train->track_graph[branch_ind].switch_dir;
        if( switch_dir == DIR_LEFT ) {
          set_switch( train->next_branch_num, CURVED, train->switch_states );
          int other_switch_num = NONE;
          switch( train->next_branch_num ) {
          case 154:
            other_switch_num = 153;
            break;
          case 156:
            other_switch_num = 155;
            break;
          }
          if( other_switch_num != NONE ) {
            set_switch( other_switch_num, STRAIGHT, train->switch_states );
          }
        } else if( switch_dir == DIR_RIGHT ) {
          set_switch( train->next_branch_num, STRAIGHT, train->switch_states );
        }
        rail_msg.general_val = UPDATE_POS;
        break;
      }
    // TODO: Print controls
    case TR_USER_CTRL:
      train->user_controlled = true;
      train->priority = LIMO;
      print_commands( train->train_id );
      break;
    case TR_STOP_USER_CTRL:
      train->user_controlled = false;
      train->priority = SHUTTLE;
      remove_print_commands( );
      break;
    case TR_PRIORITY:
      train->user_controlled = false;
      train->priority = train_cmd_args.speed_num;
      if( train->priority == TANK ) {
        train->dest_id = 70;
        train->mm_past_dest = 0;
        sensor_id_to_name( train->dest_id, sensor_name );
        Printf( COM2, "\0337\033[14;%dHCurrent destination: %c%c%c\0338", get_train_idx( train->train_id ) * 37, sensor_name[0], sensor_name[1], sensor_name[2] );
      }
      break;
    case TR_SET_RAND_DEST:
      train->dest_id = get_rand_dest( train->time_since_last_pos_update, train->track_graph, train->prev_sensor_id );
      sensor_id_to_name( train->dest_id, sensor_name );
      Printf( COM2, "\0337\033[14;%dHCurrent destination: %c%c%c\0338", get_train_idx( train->train_id ) * 37, sensor_name[0], sensor_name[1], sensor_name[2] );
      break;
    default:
      break;
    }
  }
}

void switch_exe_worker( ) {
  rail_msg_t rail_msg;
  rail_msg.request_type = SWITCH_EXE_READY;
  rail_msg.to_server_content.nullptr = NULL;
  rail_msg.from_server_content.nullptr = NULL;
  int ret_val;
  int cmd_server_tid;
  int switch_num; // same as switch_id
  int state;
  int *switch_states;
  track_node_t *graph;
  switch_cmd_args_t switch_cmd_args;
  ret_val = Receive( &cmd_server_tid, (char *)&switch_num, sizeof( switch_num ) );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  ret_val = Reply( cmd_server_tid, (char *)&rail_msg, 0 );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  rail_msg.train_or_switch_id = switch_num;

  FOREVER {
    ret_val = Send( cmd_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&switch_cmd_args, sizeof(switch_cmd_args) );
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
  int cmd_server_tid;
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
  ret_val = Receive( &cmd_server_tid, (char *)&update_train_args, sizeof( update_train_args ) );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  trains = update_train_args.trains;
  ret_val = Reply( cmd_server_tid, (char *)&trains, 0 );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );
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
        if( cur_time - trains[i].time_at_last_landmark > 1500 && 
            trains[i].cur_speed != 0 && 
            cur_time - trains[i].init_time > 2500 &&
            !(trains[i].user_controlled) ) {
          ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
          ((rail_msg.to_server_content).rail_cmds)->train_action = TR_INIT;
          ret_val = Send( cmd_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
          Printf( COM2, "\0337\033[1A\033[2K\rUh oh... we lost Train %d. Reinitializing.\0338", trains[i].train_id );
        }
        if( trains[i].cur_speed == 0 &&
            cur_time - trains[i].speed_change_time > 500 &&
            !(trains[i].user_controlled) ) {
          ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
          ((rail_msg.to_server_content).rail_cmds)->train_action = TR_CHANGE_SPEED;
          ((rail_msg.to_server_content).rail_cmds)->train_speed = 10;
          ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
          ret_val = Send( cmd_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
        }
        /* clear destination if stopping flag is set */
        if( trains[i].dest_id == NONE && trains[i].prev_dest_id != NONE && trains[i].cur_vel == 0 ) {
          if( trains[i].set_rand_dest ) {
            trains[i].dest_id = get_rand_dest( cur_time, trains[i].track_graph, trains[i].prev_sensor_id );
            sensor_id_to_name( trains[i].dest_id, dest );
            if( !(trains[i].user_controlled) ) {
              ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
              ((rail_msg.to_server_content).rail_cmds)->train_action = TR_CHANGE_SPEED;
              ((rail_msg.to_server_content).rail_cmds)->train_speed = 10;
              ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
              ret_val = Send( cmd_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
            }
            Printf( COM2, "\0337\033[14;%dHCurrent destination: %c%c%c\0338", i * 37, dest[0], dest[1], dest[2] );
          } else if( trains[i].priority == TANK ) {
            switch( trains[i].prev_dest_id ) {
            case 39:
              trains[i].dest_id = 15;
              trains[i].mm_past_dest = -100;
              break;
            case 15:
              trains[i].dest_id = 70;
              trains[i].mm_past_dest = 0;
              break;
            case 70:
              trains[i].dest_id = 56;
              trains[i].mm_past_dest = 100;
              break;
            case 56:
              trains[i].dest_id = 39;
              trains[i].mm_past_dest = 250;
              ((rail_msg.to_server_content).rail_cmds)->rail_cmd_switch_idx = -1;
              if( trains[i].switch_states[7] == SW_STRAIGHT ) {
                set_switch( 7, CURVED, trains[i].switch_states );
              }
              if( trains[i].switch_states[18] == SW_CURVED ) {
                set_switch( 18, STRAIGHT, trains[i].switch_states );
              }
              break;
            }
            sensor_id_to_name( trains[i].dest_id, dest );
            ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
            ((rail_msg.to_server_content).rail_cmds)->train_action = TR_CHANGE_SPEED;
            ((rail_msg.to_server_content).rail_cmds)->train_speed = 10;
            ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
            ret_val = Send( cmd_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
            Printf( COM2, "\0337\033[14;%dHCurrent destination: %c%c%c\0338", i * 37, dest[0], dest[1], dest[2] );
          } else {
            Printf( COM2, "\0337\033[14;%dHCurrent destination: N/A\0338", i * 37 );
          }
        }
        if( trains[i].user_controlled ) {
          trains[i].next_branch_num = get_next_branch_num( &(trains[i]) );
          if( trains[i].cur_vel == 0 && trains[i].dest_id != NONE ) {
            bool has_reached_dest = reached_dest( &(trains[i]) );
            if( has_reached_dest ) {
              trains[i].dest_id = get_rand_dest( cur_time, trains[i].track_graph, trains[i].prev_sensor_id );
              sensor_id_to_name( trains[i].dest_id, dest );
              Printf( COM2, "\0337\033[14;%dHCurrent destination: %c%c%c\0338", i * 37, dest[0], dest[1], dest[2] );
            }
          }
        }
      }
    }
    i = cur_time % TR_MAX;
    if( trains[i].state != NOT_INITIALIZED && trains[i].state != INITIALIZING ) {
      ret_val = update_track_reservation( &(trains[i]), trains );
      if( ret_val < 0 && !trains[i].warning_displayed ) {
        if( trains[i].user_controlled ) {
          trains[i].warning_displayed = true;
          print_stop_sign( );// display a big stop sign
        }
        else { 
          // can display system train colision warning here
        }
      }
      if( ret_val == 0 && trains[i].warning_displayed ) {
        if( trains[i].user_controlled ) {
          trains[i].warning_displayed = false;
          erase_stop_sign( );
        }
        else {
          // can erase system train collision warnings here
        }
      }
      if( !(trains[i].user_controlled) && ( trains[i].state == READY || trains[i].state == HANDLING_COLLISION ) ) {
        if( ret_val == -1 ) {
          //Printf( COM2, "Train %d needs issuing reverse\r\n", trains[i].train_id );
          ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
          ((rail_msg.to_server_content).rail_cmds)->train_action = TR_REVERSE;
          ((rail_msg.to_server_content).rail_cmds)->train_speed = -1;
          ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
          ret_val = Send( cmd_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
          ((rail_msg.to_server_content).rail_cmds)->train_action = TR_CHANGE_SPEED;
          ((rail_msg.to_server_content).rail_cmds)->train_speed = 10;
          ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
          ret_val = Send( cmd_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
        } else if( ret_val == -2 && trains[i].cur_speed != 9 && trains[i].cur_speed != 24 ) {
          if( trains[i].cur_speed != 0 || trains[i].cur_vel == 0 ) {
            //Printf( COM2, "Train %d needs issuing slow down\r\n", trains[i].train_id );
            ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
            ((rail_msg.to_server_content).rail_cmds)->train_action = TR_CHANGE_SPEED;
            ((rail_msg.to_server_content).rail_cmds)->train_speed = 9;
            ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
            ret_val = Send( cmd_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
          }
        } else if( ret_val == -3 && trains[i].cur_speed != 0 ) {
          //Printf( COM2, "Train %d needs issuing stop\r\n", trains[i].train_id );
          ((rail_msg.to_server_content).rail_cmds)->train_id = trains[i].train_id;
          ((rail_msg.to_server_content).rail_cmds)->train_action = TR_STOP;
          ((rail_msg.to_server_content).rail_cmds)->train_speed = 0;
          ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
          ret_val = Send( cmd_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
        }
      }
      if( ret_val > 0 ) {
        //Printf( COM2, "Train %d needs issuing speed up\r\n", ret_val );
        int ot_idx = get_train_idx( ret_val );
        if( trains[ot_idx].cur_speed != 11 && trains[ot_idx].cur_speed != 26 && !(trains[ot_idx].user_controlled) && trains[ot_idx].state == READY ) {
          ((rail_msg.to_server_content).rail_cmds)->train_id = ret_val;
          ((rail_msg.to_server_content).rail_cmds)->train_action = TR_CHANGE_SPEED;
          ((rail_msg.to_server_content).rail_cmds)->train_speed = 11;
          ((rail_msg.to_server_content).rail_cmds)->train_delay = 0;
          ret_val = Send( cmd_server_tid, (char *)&rail_msg, sizeof(rail_msg), (char *)&ret_val, 0 );
        }
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
  int cur_speed_normalized;
  update_train_args_t update_train_args;
  ret_val = Receive( &rail_server_tid, (char *)&update_train_args, sizeof( update_train_args ) );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  trains = update_train_args.trains;
  ret_val = Reply( rail_server_tid, (char *)&trains, 0 );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );
  FOREVER {
    cur_time = Delay( 10 );
    for( i = 0; i < TR_MAX; ++i ) {
      if( trains[i].state != NOT_INITIALIZED && trains[i].state != INITIALIZING ) {
        cur_speed_normalized = trains[i].cur_speed > 14 ? trains[i].cur_speed - 15 : trains[i].cur_speed;
        Printf( COM2, "\0337\033[15;%dH%d    \0338", ( i * 37 ) + 7, cur_speed_normalized );
        Printf( COM2, "\0337\033[18;%dH%d    \0338", ( i * 37 ) + 9, trains[i].mm_past_landmark / 10 );
        Printf( COM2, "\0337\033[16;%dH%d    \0338", ( i * 37 ) + 9, trains[i].cur_vel );
        //Printf( COM2, "\0337\033[12;%dH%d    \0338", ( i * 37 ) + 16, trains[i].next_branch_num );
        /*if( cur_time % 50 < 10 ) {
          print_rsv( &(trains[i]), trains );
        }*/
      } else if( trains[i].state == INITIALIZING ) {
        Printf( COM2, "\0337\033[13;%dH===== TRAIN %d ====\0338", ( i * 37 ), trains[i].train_id );
        Printf( COM2, "\0337\033[15;%dHSpeed: N/A\0338", ( i * 37 ) );
        Printf( COM2, "\0337\033[18;%dHmm past: N/A\0338", ( i * 37 ) );
        Printf( COM2, "\0337\033[16;%dHCur vel: N/A\0338", ( i * 37 ) );
        //Printf( COM2, "\0337\033[12;%dHNext branch num: N/A\0338", ( i * 37 ) );
      }
    }
  }
}

/* return inserted idx, if empty spot not found, return NONE */
inline int insert_cmd( generic_cmds_list_t * cmds_list, int id, int action, int delay, int train_speed, int train_dest, int train_mm_past_dest, int train_accel, int train_decel ) {
  if( cmds_list->count >= CMD_QUEUE_MAX ) {
    assertum( 1, false, "ERROR: don't panic, this indicates exe worker queues are overrun, just increase the cmd queue size. train:%d cmd: %d", id, action );
    return NONE;
  }

  int idx = 0;
  for( ; idx < CMD_QUEUE_MAX; ++idx ) {
    if( cmds_list->cmds[idx].id == NONE ) {
      assertu( 1, cmds_list->cmds[idx].action == NONE && cmds_list->cmds[idx].delay == NONE );
      cmds_list->cmds[idx].id = id;
      cmds_list->cmds[idx].action = action;
      cmds_list->cmds[idx].delay = delay;
      cmds_list->cmds[idx].train_speed = train_speed;
      cmds_list->cmds[idx].train_dest = train_dest;
      cmds_list->cmds[idx].train_mm_past_dest = train_mm_past_dest;
      cmds_list->cmds[idx].train_accel = train_accel;
      cmds_list->cmds[idx].train_decel = train_decel;
      ++( cmds_list->count );
      assertu( 1, cmds_list->count <= CMD_QUEUE_MAX && cmds_list->count >= 0 );
      return idx;
    }
  }
  assertum( 1, false, "BUG: should not reach here, this indicates cmds_list counter is wrong" );
  return NONE;
}

/* return the idx of item with the smallest delay, NONE of empty list */
inline int extract_cmd( generic_cmds_list_t* cmds_list, int* id, int* action, int* delay, int* train_speed, int* train_dest, int* train_mm_past_dest, int* train_accel, int* train_decel ) {
  *id = *action = *delay = *train_speed = *train_dest = *train_mm_past_dest = *train_accel = *train_decel = NONE;
  int cur_time = Time( );
  int idx = 0;
  int smallest_delay_so_far = INT_MAX;
  int ret_idx = NONE;
  for( ; idx < CMD_QUEUE_MAX; ++idx ) {
    if( cmds_list->cmds[idx].id != NONE && cmds_list->cmds[idx].delay < smallest_delay_so_far ) {
      assert( 1, cmds_list->cmds[idx].action != NONE );// && cmds_list->cmds[idx].delay != NONE );
      smallest_delay_so_far = cmds_list->cmds[idx].delay;
      *id = cmds_list->cmds[idx].id;
      *action = cmds_list->cmds[idx].action;
      *delay = cmds_list->cmds[idx].delay - cur_time > 0 ? cmds_list->cmds[idx].delay - cur_time : 0;

      *train_speed = cmds_list->cmds[idx].train_speed;
      *train_dest = cmds_list->cmds[idx].train_dest;
      *train_mm_past_dest = cmds_list->cmds[idx].train_mm_past_dest;
      *train_accel = cmds_list->cmds[idx].train_accel;
      *train_decel = cmds_list->cmds[idx].train_decel;
      ret_idx = idx;
    }
  }

  if( ret_idx != NONE ) {
    cmds_list->cmds[ret_idx].id = NONE;
    cmds_list->cmds[ret_idx].action = NONE;
    cmds_list->cmds[ret_idx].delay = NONE;

    cmds_list->cmds[ret_idx].train_speed = NONE;
    cmds_list->cmds[ret_idx].train_dest = NONE;
    cmds_list->cmds[ret_idx].train_mm_past_dest = NONE;
    cmds_list->cmds[ret_idx].train_accel = NONE;
    cmds_list->cmds[ret_idx].train_decel = NONE;

    --( cmds_list->count );
  }
  assert( 1, cmds_list->count <= CMD_QUEUE_MAX && cmds_list->count >= 0 );
  assert( 1, ( ret_idx == NONE && *id == NONE && *action == NONE ) 
          || ( ret_idx != NONE && *id != NONE && *action != NONE ));
  return ret_idx;
}

inline void init_generic_cmds_lists( generic_cmds_list_t** array_of_cmds_list, int num_max ) {
  int i = 0;
  for( ; i < num_max; ++i ) {
    array_of_cmds_list[i]->count = 0;
    int j = 0;
    for( ; j < CMD_QUEUE_MAX; ++j ) {
      ((array_of_cmds_list[i])->cmds)[j].id = NONE;
      ((array_of_cmds_list[i])->cmds)[j].action = NONE;
      ((array_of_cmds_list[i])->cmds)[j].delay = NONE;

      ((array_of_cmds_list[i])->cmds)[j].train_speed = NONE;
      ((array_of_cmds_list[i])->cmds)[j].train_dest = NONE;
      ((array_of_cmds_list[i])->cmds)[j].train_mm_past_dest = NONE;
      ((array_of_cmds_list[i])->cmds)[j].train_accel = NONE;
      ((array_of_cmds_list[i])->cmds)[j].train_decel = NONE;
    }
  }
}

inline void init_bool_array( bool* bool_array, int size ) {
  int i = 0;
  for( ; i < size; ++i ) {
    bool_array[i] = false;
  }
}

void cmd_server( ) {
  /* initialize workers queue */
  bool switch_exe_busy[SW_MAX];
  generic_cmds_list_t* switch_cmds[SW_MAX]; // idx 0 - 22, 1-22 are used, 0 is empty
  generic_cmds_list_t switch0_cmds; // this is never used;
  generic_cmds_list_t switch1_cmds;
  generic_cmds_list_t switch2_cmds;
  generic_cmds_list_t switch3_cmds;
  generic_cmds_list_t switch4_cmds;
  generic_cmds_list_t switch5_cmds;
  generic_cmds_list_t switch6_cmds;
  generic_cmds_list_t switch7_cmds;
  generic_cmds_list_t switch8_cmds;
  generic_cmds_list_t switch9_cmds;
  generic_cmds_list_t switch10_cmds;
  generic_cmds_list_t switch11_cmds;
  generic_cmds_list_t switch12_cmds;
  generic_cmds_list_t switch13_cmds;
  generic_cmds_list_t switch14_cmds;
  generic_cmds_list_t switch15_cmds;
  generic_cmds_list_t switch16_cmds;
  generic_cmds_list_t switch17_cmds;
  generic_cmds_list_t switch18_cmds;
  generic_cmds_list_t switch153_cmds;
  generic_cmds_list_t switch154_cmds;
  generic_cmds_list_t switch155_cmds;
  generic_cmds_list_t switch156_cmds;
  switch_cmds[0] = &switch0_cmds;
  switch_cmds[1] = &switch1_cmds;
  switch_cmds[2] = &switch2_cmds;
  switch_cmds[3] = &switch3_cmds;
  switch_cmds[4] = &switch4_cmds;
  switch_cmds[5] = &switch5_cmds;
  switch_cmds[6] = &switch6_cmds;
  switch_cmds[7] = &switch7_cmds;
  switch_cmds[8] = &switch8_cmds;
  switch_cmds[9] = &switch9_cmds;
  switch_cmds[10] = &switch10_cmds;
  switch_cmds[11] = &switch11_cmds;
  switch_cmds[12] = &switch12_cmds;
  switch_cmds[13] = &switch13_cmds;
  switch_cmds[14] = &switch14_cmds;
  switch_cmds[15] = &switch15_cmds;
  switch_cmds[16] = &switch16_cmds;
  switch_cmds[17] = &switch17_cmds;
  switch_cmds[18] = &switch18_cmds;
  switch_cmds[19] = &switch153_cmds;
  switch_cmds[20] = &switch154_cmds;
  switch_cmds[21] = &switch155_cmds;
  switch_cmds[22] = &switch156_cmds;

  bool train_exe_busy[TRAIN_MAX];
  generic_cmds_list_t* train_cmds[TRAIN_MAX];
  generic_cmds_list_t train0_cmds;
  generic_cmds_list_t train1_cmds;
  generic_cmds_list_t train2_cmds;
  generic_cmds_list_t train3_cmds;
  train_cmds[0] = &train0_cmds;
  train_cmds[1] = &train1_cmds;
  train_cmds[2] = &train2_cmds;
  train_cmds[3] = &train3_cmds;

  init_generic_cmds_lists( switch_cmds, SW_MAX );
  init_generic_cmds_lists( train_cmds, TRAIN_MAX );
  init_bool_array( switch_exe_busy, SW_MAX );
  init_bool_array( train_exe_busy, TRAIN_MAX );
  
  /* initialize fundamental variables */
  int i, ret_val, client_tid, rail_server_tid;
  i = ret_val = client_tid = rail_server_tid = 0;
  train_state_t * trains = NULL;
  int * switch_states;
  track_node_t * track_graph;
  char sensor_name[4];
  rail_server_tid = MyParentTid( );
  assertu( 1, rail_server_tid > 0 );

  /* get trains, switches and graph from rail_server */
  ret_val = Receive( &client_tid, (char *)&trains, sizeof( train_state_t* ));
  debugu(1, "received trains pointer: %x", trains );
  assertum( 1, rail_server_tid == client_tid && ret_val >= 0, "retval: %d", ret_val );
  switch_states = trains[0].switch_states;
  track_graph = trains[0].track_graph;

  /* Train exe workers */
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

  /* Switch exe workers */
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

  /* update_trains woker */
  int update_trains_tid = Create( 13, &update_trains );
  update_train_args_t update_train_args;
  update_train_args.trains = trains;
  ret_val = Send( update_trains_tid, (char *)&update_train_args, sizeof( update_train_args ), (char *)&client_tid, 0 );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );

  /* print_trains worker */
  int print_trains_tid = Create( 14, &print_trains );
  ret_val = Send( print_trains_tid, (char *)&update_train_args, sizeof( update_train_args ), (char *)&client_tid, 0 );
  assertm( 1, ret_val >= 0, "retval: %d", ret_val );

  rail_msg_t receive_msg;
  rail_cmds_t* receive_cmds; 

  /* register self as cmd_server*/
  if( RegisterAs(( char* )CMD_SERVER ) == -1 ) {
    bwputstr( COM2, "ERROR: failed to register cmd_server, aborting ...\n\r" );
    Exit( );
  }
  ret_val = Reply( rail_server_tid, (char *)&client_tid, 0 );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );

  /* helper functions for the cmds waiting queue */
  inline void insert_train_cmd( ) {
    assert( 1, receive_cmds );
    int train_idx = convert_train_num2idx( receive_cmds->train_id );
    assertum( 1, train_idx != NONE, "failure here means a train num is not given/mapped with an idx" );
    train_exe_worker_tid = train_exe_worker_tids[train_idx];
    insert_cmd( train_cmds[train_idx], 
        receive_cmds->train_id,    receive_cmds->train_action,
        receive_cmds->train_delay, receive_cmds->train_speed,
        receive_cmds->train_dest,  receive_cmds->train_mm_past_dest,
        receive_cmds->train_accel, receive_cmds->train_decel );
  }

  inline void insert_switch_cmd( int cur_rail_cmd_switch_idx, bool user_command ) {
    assert( 1, receive_cmds );
    int switch_cmds_idx = receive_cmds->switch_cmds[cur_rail_cmd_switch_idx].switch_id;//idx is the same as id
    assertu( 1, switch_cmds_idx > 0 && switch_cmds_idx < SW_MAX );
    insert_cmd( switch_cmds[switch_cmds_idx], switch_cmds_idx, 
        receive_cmds->switch_cmds[cur_rail_cmd_switch_idx].switch_action, 
        receive_cmds->switch_cmds[cur_rail_cmd_switch_idx].switch_delay, NONE, NONE, NONE, NONE, NONE );

    int other_switch_idx = get_switch_twin( switch_cmds_idx );
    if( user_command && other_switch_idx != NONE ) {
      if( receive_cmds->switch_cmds[cur_rail_cmd_switch_idx].switch_action == SW_CURVED ) {
        assertum( 1, other_switch_idx > 0 && other_switch_idx < SW_MAX, "other switch_idx: %d", other_switch_idx );
        insert_cmd( switch_cmds[other_switch_idx], other_switch_idx, SW_STRAIGHT, 
            receive_cmds->switch_cmds[cur_rail_cmd_switch_idx].switch_delay, NONE, NONE, NONE, NONE, NONE ); // same delay as twin
      } else if ( receive_cmds->switch_cmds[cur_rail_cmd_switch_idx].switch_action == SW_STRAIGHT ) {
        assertum( 1, other_switch_idx > 0 && other_switch_idx < SW_MAX, "other switch_idx: %d", other_switch_idx );
        insert_cmd( switch_cmds[other_switch_idx], other_switch_idx, SW_CURVED, 
            receive_cmds->switch_cmds[cur_rail_cmd_switch_idx].switch_delay, NONE, NONE, NONE, NONE, NONE ); // same delay as twin
      }
    }
  }

  FOREVER { 
    ret_val = Receive( &client_tid, (char *)&receive_msg, sizeof( receive_msg ));
    debugu( 1, "cmd_server received client_tid: %d, request_type: %d", client_tid, receive_msg.request_type );
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
    receive_cmds = receive_msg.to_server_content.rail_cmds;

    debugu( 1, "cmd_server before switch cases" );
    switch( receive_msg.request_type ) {
      case COLLISION_CMDS:
      case USER_INPUT:
        debugu( 1, "USER_INPUT OR COLLISION" );
        // TODO: If user/collision reverse, make sure we properly set switches ahead so we don't derail
        ret_val = Reply( client_tid, (char *)&receive_msg, 0 );
        assertum( 1, ret_val >= 0, "retval: %d", ret_val );
        if( receive_cmds->train_id != NONE ) {
          insert_train_cmd( );
        } else if ( receive_cmds->rail_cmd_switch_idx != NONE ) {
            assertum( 1, receive_cmds->rail_cmd_switch_idx == 0, "user packed %d sw_cmds, should it?", receive_cmds->rail_cmd_switch_idx + 1 );
            insert_switch_cmd( 0, true );
        } else if ( receive_msg.to_server_content.rail_cmds->rsv_node_id != NONE ) {
          int rsv_node_id = receive_msg.to_server_content.rail_cmds->rsv_node_id;
          int rsv_node_dir = receive_msg.to_server_content.rail_cmds->rsv_node_dir;
          track_edge_t *edge = &(track_graph[rsv_node_id].edge[rsv_node_dir]);
          if( edge->middle_train_num == USER_INPUT_NUM || edge->begin_train_num == USER_INPUT_NUM ) {
            edge->middle_train_num = NONE;
            edge->begin_train_num = NONE;
            Printf( COM2, "\0337\033[1A\033[2K\rUnreserved track from %s, direction %d\0338", 
                track_graph[rsv_node_id].name, rsv_node_dir );
          } else if( edge->middle_train_num == NONE && edge->begin_train_num == NONE ) {
            edge->middle_train_num = USER_INPUT_NUM;
            edge->begin_train_num = USER_INPUT_NUM;
            Printf( COM2, "\0337\033[1A\033[2K\rReserved track from %s, direction %d\0338", 
                track_graph[rsv_node_id].name, rsv_node_dir );
          } else {
            Printf( COM2, "\0337\033[1A\033[2K\rCould not reserve track from %s, direction %d\0338", 
                track_graph[rsv_node_id].name, rsv_node_dir );
          }
        }
        break;

      case RAIL_CMDS:
        debugu( 1, "RAIL_CMDS" );
        /* get and store train cmds */
        if( receive_cmds->train_id != NONE ) {
          insert_train_cmd( );
        }
        /* get and store switch cmds */
        for( i = 0; i <= receive_cmds->rail_cmd_switch_idx; ++i ) {
          debugu( 4, "before reply to switch_exe_worker: switch_idx: %d, i: %d", receive_cmds->rail_cmd_switch_idx, i );
          insert_switch_cmd( i, false );
        }
        break;

      case TRAIN_EXE_READY:
        {
          int train_idx = convert_train_num2idx( receive_msg.train_or_switch_id );
          assertu( 1, train_idx != NONE );
          train_exe_busy[train_idx] = false;
          if( receive_msg.general_val == UPDATE_POS ) {
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
          } else if( receive_msg.general_val != NONE ) {
            train_idx = convert_train_num2idx( receive_msg.general_val );
            assertu( 1, train_idx != NONE );
            train_state_t *train = &(trains[train_idx]);
            if( train->state != NOT_INITIALIZED && train->state != INITIALIZING ) {
              update_prev_sensor_id_for_rev( train );
              train->rev_branch_ignore = NONE;
            }
          }
        }
        break;
      case SWITCH_EXE_READY: {
        debugu( 1, "SWITCH EXE" );
        int switch_idx = receive_msg.train_or_switch_id;
        CONVERT_SWITCH_ID( switch_idx );
        assertu( 1, switch_idx > 0 && switch_idx < SW_MAX ); //153-156
        switch_exe_busy[switch_idx] = false;
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
      }
        break;
      default:
        assertum( 1, false, "ERROR: cmd_server wrong request_type: %d was sent to cmd_server by tid: %d", 
            receive_msg.request_type, client_tid );
        break;
    }

    /* after switch cases, dispatch exe workers now */
    debugu( COM2, "before dispatching workers" );
    int id, action, delay, train_speed, train_dest, train_mm_past_dest, train_accel, train_decel;
    for( i = 0; i < TRAIN_MAX; ++i ) {
      if( train_cmds[i]->count > 0 && train_exe_busy[i] == false ) {
        extract_cmd( train_cmds[i], &id, &action, &delay, &train_speed, &train_dest, &train_mm_past_dest, &train_accel, &train_decel );
        assertu( 1, i == convert_train_num2idx( id )); // really shouldn't fail here
        train_exe_worker_tid = train_exe_worker_tids[i];
        assertu( 1, train_exe_worker_tid > 0 );
        train_cmd_args.cmd = action;
        train_cmd_args.delay_time = delay;
        train_cmd_args.speed_num = train_speed;
        train_cmd_args.dest = train_dest;
        train_cmd_args.mm_past_dest = train_mm_past_dest;
        train_cmd_args.accel_rate = train_accel;
        train_cmd_args.decel_rate = train_decel;

        train_exe_busy[i] = true;
        ret_val = Reply( train_exe_worker_tid, (char *)&train_cmd_args, sizeof( train_cmd_args ) );
        assertum( 1, ret_val == 0, "ret_val: %d", ret_val );
      }
    }

    for( i = 1; i < SW_MAX; ++i ) { // recall idx=0 is not used in switch array
      if( switch_cmds[i]->count > 0 && switch_exe_busy[i] == false ) {
        extract_cmd( switch_cmds[i], &id, &action, &delay, &train_speed, &train_dest, &train_mm_past_dest, &train_accel, &train_decel );
        assert( 1, i == id );
        switch_exe_worker_tid = switch_exe_worker_tids[i];
        assert( 1, switch_exe_worker_tid > 0 );
        switch_cmd_args.state = action;
        switch_cmd_args.delay_time = delay;

        switch_exe_busy[i] = true;
        ret_val = Reply( switch_exe_worker_tid, (char *)&switch_cmd_args, sizeof( switch_cmd_args ) );
        assertm( 1, ret_val >= 0, "retval: %d", ret_val );
      }
    }
  }
}


void rail_server( ) {
  if( RegisterAs( (char*)RAIL_SERVER ) == -1 ) {
    bwputstr( COM2, "ERROR: failed to register rail_server, aborting ...\n\r" );
    Exit( );
  }
  int client_tid, ret_val, i;

  /* track and trainx initialization */
  track_node_t track_graph[TRACK_MAX];
  //init_tracka( (track_node_t*)track_graph );
  init_trackb( (track_node_t*)track_graph );
  int switch_states[SW_MAX];
  train_state_t trains[TR_MAX];
  train_state_t *p_trains = trains;
  init_trains( trains, (track_node_t*)track_graph, switch_states );
  init_switches( switch_states );

  /* create cmd_server, other workers will need to know cmd_server_tid */
  int cmd_server_tid = Create( 3, &cmd_server );
  assertu( 1, cmd_server_tid > 0 );
  ret_val = Send( cmd_server_tid, (char *)&p_trains, sizeof( train_state_t* ), (char *)&cmd_server_tid, 0 );
  assertum( 1, ret_val >= 0, "retval: %d", ret_val );

  int parse_user_input_tid = Create( 6, &parse_user_input ); // 45
  debug( 2,  "User input task tid: %d", parse_user_input_tid );

  /* graph_search_workers */
  int rail_graph_worker_tids[TR_MAX]; 
  for( i = 0; i < TR_MAX; ++i ) {
    rail_graph_worker_tids[i] = Create( 11, &rail_graph_worker );
    assertu( 1, rail_graph_worker_tids[i] > 0 );
  }

  /* Sensor worker and courier declarations */
  int worker_tid;
  int sensor_courier_tid = Create( 2, &sensor_data_courier );
  assertu( 1, sensor_courier_tid > 0 );

  declare_ring_queue( int, sensor_workers, SENSOR_WORKER_MAX );
  int sensor_worker_tids[SENSOR_WORKER_MAX];
  //char sensor_name[4];
  for( i = 0; i < SENSOR_WORKER_MAX; ++i ) {
    sensor_worker_tids[i] = Create( 10, &sensor_worker );
    assertu( 1, sensor_worker_tids[i] > 0 );
    ret_val = Send( sensor_worker_tids[i], (char *)&client_tid, 0, (char *)&client_tid, 0 );
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
  }
  sensor_args_t sensor_args;
  sensor_args.trains = trains;

  rail_msg_t receive_msg;
  FOREVER { 
    ret_val = Receive( &client_tid, (char *)&receive_msg, sizeof( rail_msg_t ));
    assertum( 1, ret_val >= 0, "retval: %d", ret_val );
    switch( receive_msg.request_type ) {
      case SENSOR_DATA:
        {
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
     case SENSOR_WORKER_READY: // worker responds with the train that hit the sensor
        // get train, graph search
        sensor_workers_push_back( client_tid );
        if( receive_msg.to_server_content.train_state != NULL ) {
          // FIXME: reply to the trigger the first
          // NOTE: Does the above matter? Reply is non-blocking. We're saving only a few nanoseconds by doing so.
          int i = 0;
          for( ;  i < TR_MAX ; ++i ) {
            if( trains[i].dest_id != NONE && trains[i].state == READY ) {
              train_state_t * train_to_send = &(trains[i]);
              if( !(train_to_send->user_controlled) ) {
                ret_val = Reply( rail_graph_worker_tids[i], (char*)&( train_to_send ), sizeof( train_to_send ));
                debugu( 1, "rail_server just replied to the graph worker" );
                assertum( 1, ret_val == 0, "ERROR: ret_val: %d, tid: %d, failure here means graph worker is not ready", 
                    ret_val, rail_graph_worker_tids[i] );
              }
            }
          }
        }
        break;
      default:
        assertum( 1, false, "ERROR: rail_server: unrecognized request: %d", receive_msg.request_type );
        break;
    }
  }
}

