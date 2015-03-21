#include "rail_control.h"
#include "tools.h"
#include "track_data_new.h"
#include "track_node.h"
#include "global.h"
#include "io.h" 
#include "ring_buf.h"
#include "rail_helper.h"

/* get_next_command calls graph search to get the shortest path,
 * for now, if the path only has length of one, the we issue stop command.
 * if there are more than one node, it finds out if there needs reverse/switch
 * If two nodes are reverse of each other,
 * we issue reverse command. If the first node is MR, we check the second
 * to decide which direction to go to. 
 */

/* NOTES on get_next_command work flow:
 * we are sitting on the first sensor, we set the prev_sensor to be the one we
 * are sitting on. We update the prev_sensor when we see another one. For each
 * branch, we check:
 * if prev_sensor == src_id: 
 *    if dist < safe_branch_dist, 
 *       do not do anything
 *    if dist >= safe_branch dist, 
 *        branch it
 * if prev_sensor == second_sensor after src_sensor:
 *    if dist < safe_branch_dist 
 *        branch it we wouldn't have time later
 *    if dist >= safe_branch_dist
 *        do not do anything
 *
 * overall:
 * if (( prev_sensor == src_id && dist >= safe_branch_dist ) || 
 *       prev_sensor == second_sensor && dist < safe_branch dist ) 
 *       switch turn out
 * else
 *  do nothing
 */


inline void init_rail_cmds( rail_cmds_t* cmds ) {
  cmds->switch_idx = NONE;
  cmds->train_id = cmds->train_action = cmds->train_delay = cmds->train_speed = NONE;
  int i = 0;
  for( ; i < SW_CMD_MAX; ++i ) {
    cmds->switch_cmds[i].switch_id = NONE;
    cmds->switch_cmds[i].switch_action = NONE;
    cmds->switch_cmds[i].switch_delay = NONE;
  }
}

//TODO: call static search after switch comes back 
//TODO: put this dynamic checking into a separate path
/* note for prediction:
 * 1. sensor_worker gets sensor byte 
 * 2. call static prediction
 * 3. run graph search, graph search returns cmds and next expected path 
 * 4. issue commands first, this will update switch states used for static prediction in the future
 * 5. if before next sensor there are reverses 
 *      since the graph gives one reverse at a time, we find the reverse node, remaining distance after reverse node by
 *      get_cur_stopping_distance( train );
 *      here we need to handle two things:
 *        a. sensors hit before it stops, we put these sensors as a list of expected hits before it stops
 *           discard the sensor when it's hit, and discard all (even if unhit) when the train hits any reverse of them
 *        b. every merge we pass by, we need to examine the branches and determine if we need to switch them, and when
 *           note that this step also generates commands but only for switches
 *           when looking backward, if the merge is from the graph search, we do not check it's turnout
 *    if before next sensor there is a stop command
 *      we get the remaining stopping distance after the destination by: 
 *      get_cur_stopping_distance( train );
 *      we repeat 5.a, for each expected sensor hits, we store them in a list
 *    if there is no reverse nor stop 
 *      do not do anything, can assert on: recalculate a result, should be the same as the one calculated at step 2
 */

// predict one sensor at a time, this function is called when the train is in reversing or stopping state
// prbly need another function for switches tracking
//void predict_next_sensor_dynamic( train_state_t* train_state, rail_cmds_t* rail_cmds ) {
//  //assertu( 1, train_state
//  ///* reversing case */
//  //if( train_state->state == REVERSING ) {
//  //  train_state->rv_original_expected_sensor = train_state->next_sensor_id; 
//  //  int stop_dist = get_cur_stopping_distance( train_state );
//  //}
//
//  ///* stopping case */
//  //else if( train_state->cur_speed == TR_STOP ) {
//
//  //}
//  //else {
//  //  assertum( 1, false, "calling predict_next_sensor_dynamic in unnecessary condition" );
//  //}
//
//}

void predict_next_sensor_dynamic( train_state_t* train_state ) {
  assertu( 1, train_state->cur_speed == 0 || train_state->state == REVERSING );
  track_node_t* cur_node = &( train_state->track_graph[train_state->prev_sensor_id] );
  assertu( 1, cur_node );
  assertu( 1, cur_node->type == NODE_SENSOR );
  int next_sensor_id = NONE;
  int next_sensor_dist = cur_node->edge[DIR_AHEAD].dist;
  int branch_ind;
  cur_node = cur_node->edge[DIR_AHEAD].dest;
  //int stop_dist = get_cur_stopping_distance( train_state );
  int stop_dist = get_cur_stopping_distance( train_state );
  while( next_sensor_id < 0 && next_sensor_dist <= stop_dist ) {
    debugu( 4, "whileloop, next_sensor_dist: %d, stop_dist: %d", next_sensor_dist, stop_dist ) ;
    if( cur_node->type == NODE_SENSOR ) {
      debugu( 4,  "1" );
      next_sensor_id = cur_node->num;
      assertu( 4, next_sensor_id >= 0 );
    }
    else if( cur_node->type == NODE_BRANCH ) {
      debugu( 4,  "2" );
      branch_ind = cur_node->num;
      if( branch_ind > 152 ) {
        branch_ind -= 134;
      }
      next_sensor_dist += cur_node->edge[train_state->switch_states[branch_ind]].dist;
      cur_node = cur_node->edge[train_state->switch_states[branch_ind]].dest;
    }
    else if( cur_node->type == NODE_EXIT ) {
      debugu( 4,  "3" );
      break;
    }
    else {
      debugu( 4,  "4" );
      next_sensor_dist += cur_node->edge[DIR_AHEAD].dist;
      cur_node = cur_node->edge[DIR_AHEAD].dest;
    }
  }

  if( next_sensor_id == NONE && train_state->state == REVERSING ) {
    debugu( 4,  "5" );
    train_state->next_sensor_id = train_state->track_graph[train_state->prev_sensor_id].reverse - train_state->track_graph;
    train_state->dist_to_next_sensor = (stop_dist * 2) - train_state->pickup_len > 0 ? (stop_dist * 2) - train_state->pickup_len : 0;
  } else if( next_sensor_id != NONE ) {
    train_state->next_sensor_id = next_sensor_id;
    train_state->dist_to_next_sensor = next_sensor_dist;
  }

}

void predict_next_sensor_static( train_state_t *train_state ) {
  track_node_t* cur_node = &(train_state->track_graph[train_state->prev_sensor_id]);
  assertu( 1, cur_node && cur_node->type == NODE_SENSOR );
  int next_sensor_id = NONE; // might be an Exit node
  int ret_node_dist = cur_node->edge[DIR_AHEAD].dist;
  int branch_ind;
  cur_node = cur_node->edge[DIR_AHEAD].dest;

  while( next_sensor_id < 0 ) {
    //Printf( COM2, "cur_node->type: %d, cur_node->num: %d\r\n", cur_node->type, cur_node->num );
    if( cur_node->type == NODE_SENSOR ) {
      //Printf( COM2, "sensor node\r\n" );
      next_sensor_id = cur_node->num; 
      assertu( 1, next_sensor_id >= 0 );
    }
    else if( cur_node->type == NODE_BRANCH ) {
     // Printf( COM2, "branch node\r\n" );
      branch_ind = cur_node->num;
      if( branch_ind > 152 ) {
        branch_ind -= 134;
      }
      //Printf( COM2, "branch_ind: %d, branch_state: %d\r\n", branch_ind, train_state->switch_states[branch_ind] );
      ret_node_dist += cur_node->edge[train_state->switch_states[branch_ind]].dist;
      cur_node = cur_node->edge[train_state->switch_states[branch_ind]].dest;
    } else if( cur_node->type == NODE_EXIT ) {
      //Printf( COM2, "exit node\r\n" );
      break;
    }
    else { /* any other node */
      //Printf( COM2, "other node\r\n" );
      ret_node_dist += cur_node->edge[DIR_AHEAD].dist;
      cur_node = cur_node->edge[DIR_AHEAD].dest;
    }
  }
  
  debugu( 4,  "dist_to_next_sensor: %d", ret_node_dist );
  debugu( 4,  "next_sensor_id: %d", next_sensor_id );
  train_state->dist_to_next_sensor = ret_node_dist;
  train_state->next_sensor_id = next_sensor_id; 
}

void predict_next_fallback_sensors_static( train_state_t *train ) {
  track_node_t* cur_node = &(train->track_graph[train->prev_sensor_id]);
  int expected_sensor_num = train->next_sensor_id;
  if( train->state == REVERSING ) {
    cur_node = cur_node->edge[DIR_AHEAD].dest;
    int branch_ind;
    while( cur_node->type != NODE_SENSOR ) {
      if( cur_node->type == NODE_BRANCH ) {
        branch_ind = cur_node->num;
        if( branch_ind > 152 ) {
          branch_ind -= 134;
        }
        cur_node = cur_node->edge[train->switch_states[branch_ind]].dest;
      } else {
        cur_node = cur_node->edge[DIR_AHEAD].dest;
      }
    }
    cur_node = cur_node->reverse;
  }
  int saw_expected_sensor = 0;
  assertu( 1, cur_node && cur_node->type == NODE_SENSOR );
  int branch_ind;
  int fallback_idx = 0;
  int i;
  int dist;
  //int cur_node_id;
  for( i = 0; i < NUM_FALLBACK; ++i ) {
    train->fallback_sensors[i] = -1;
    train->fallback_dist[i] = -1;
  }
  declare_ring_queue( track_node_t*, node_id, 10 );
  declare_ring_queue( int, node_dist, 10 );
  node_id_push_back( cur_node->edge[DIR_AHEAD].dest );
  node_dist_push_back( cur_node->edge[DIR_AHEAD].dist );

  while( !node_id_empty( ) ) {
    cur_node = node_id_pop_front( );
    dist = node_dist_pop_front( );
    //assertu( 1, node_id_top_front( ) != cur_node_id );
    if( cur_node->type == NODE_SENSOR ) {
      if( cur_node->num == expected_sensor_num ) {
        saw_expected_sensor = 1;
        node_id_push_back( cur_node->edge[DIR_AHEAD].dest );
        node_dist_push_back( dist + cur_node->edge[DIR_AHEAD].dist );
      } else {
        train->fallback_sensors[fallback_idx] = cur_node->num;
        train->fallback_dist[fallback_idx++] = dist;
      }
    }
    else if( cur_node->type == NODE_BRANCH ) {
      if( saw_expected_sensor ) {
        branch_ind = cur_node->num;
        if( branch_ind > 152 ) {
          branch_ind -= 134;
        }
        node_id_push_back( cur_node->edge[train->switch_states[branch_ind]].dest );
        node_dist_push_back( dist + cur_node->edge[train->switch_states[branch_ind]].dist );
        //assertu( 1, node_id_top_back( ) == (cur_node->edge[train->switch_states[branch_ind]].dest)->num );
      } else {
        node_id_push_back( cur_node->edge[DIR_STRAIGHT].dest );
        node_dist_push_back( dist + cur_node->edge[DIR_STRAIGHT].dist );
        //assertu( 1, node_id_top_back( ) == (cur_node->edge[DIR_STRAIGHT].dest)->num );
        node_id_push_back( cur_node->edge[DIR_CURVED].dest );
        node_dist_push_back( dist + cur_node->edge[DIR_STRAIGHT].dist );
        //assertu( 1, node_id_top_back( ) == (cur_node->edge[DIR_CURVED].dest)->num );
      }
    } else if( cur_node->type == NODE_EXIT ) {
      continue;
    } else { /* any other node */
      node_id_push_back( cur_node->edge[DIR_AHEAD].dest );
      node_dist_push_back( dist + cur_node->edge[DIR_AHEAD].dist );
      //assertu( 1, node_id_top_back( ) == (cur_node->edge[DIR_AHEAD].dest)->num );
    }
  }
}

inline void get_shortest_path( train_state_t *train ) {
  /* we run dijstra only if:
   * 1. the destination is changed, we can have an "old_dest" to specify it
   * 2. we fall on a fallback, we find out by checking the fallback_sensor_hit flag
   */
  /* train->prev_sensor_id is used as the src_id for graph search, which is the most recenlty triggered sensor */
  int all_path[NODE_MAX], all_step[NODE_MAX];
  if( true ) {//train->prev_dest_id != train->dest_id || train->fallback_sensor_hit ) { 
    debugu( 4, "runing dijkstra, train->prev_dest_id: %d, fallback_sensorhit?: %d", 
        train->prev_dest_id, train->fallback_sensor_hit );
    /* run dijkstra on the src and dest */
    dijkstra( train->track_graph, train->prev_sensor_id, all_path, (int*)( train->all_dist ), all_step );
    debugu( 4,  "dist from src to dest: %d", train->all_dist[train->dest_id] );
    
    /* get shortest path for our destination and store it in the train state */
    train->dest_path_cur_idx = 0;
    train->dest_total_steps = all_step[train->dest_id];
    debugu( 4, "train->dest_total_steps: %d", train->dest_total_steps );
    extract_shortest_path( all_path, all_step, train->prev_sensor_id, train->dest_id, (int*)( train->dest_path ));
    print_shortest_path( train->track_graph, all_path, all_step, train->prev_sensor_id, train->dest_id, (int*)( train->dest_path ));

    /* we run graph search for each destination only once, unless the fall back sensor is hit, so we update prev_dest */
    train->prev_dest_id = train->dest_id;
  }
  //else {
  //  debugu( 1, "???????running old path\n\r\n\r" );
  ///* use the memorized path, it shoud be a loop to get to the next sensor, 
  // * which should be the same as the current sensor hit, if the momorized path run out of index,
  // * TODO: need to think about when to run dijkstra next destination if there is one
  // */
  //  int i;
  //  for( i = 0; i < train->dest_total_steps; ++i ) {
  //    assertu( 1, train->dest_path[i] != NONE );
  //    debugu( 1, "src_id: %d, dest_path: %d, node: %s", src_id, train->dest_path[i], track_graph[train->dest_path[i]].name );
  //    if( src_id == train->dest_path[i] ) {
  //      train->dest_path_cur_idx = i;
  //      break;
  //    }
  //  }
  //  debugu( 1, "train->dest_total_steps: %d", train->dest_total_steps );
  //  assertu( 1, train->dest_path_cur_idx >= 0 );
  //}
  assertu( 1, train->dest_path_cur_idx >= 0 );
  assertu( 1, train->track_graph[train->dest_path_cur_idx].type == NODE_SENSOR );
}

inline void pack_train_cmd( rail_cmds_t *cmds, int train_id, int ACTION, int delay ) {
  debugu( 1, "NEWNEWNEWNEW TRAIN_CMD: train_id: %d, ACTION: %d, delay: %d", train_id, ACTION, delay );
  cmds->train_id = train_id;
  cmds->train_action = ACTION;
  cmds->train_delay = delay;
}

inline void pack_switch_cmd( rail_cmds_t *cmds, int switch_id, int ACTION, int delay ) {
  CONVERT_SWITCH_ID( switch_id );
  assertum( 1, cmds->switch_idx < SW_CMD_MAX, "failure here means we need more switch_cmd_t in rail_cmds" );
  assertum( 1, switch_id >= SW1 && switch_id <= SW156, "switch_id: %d", switch_id );
  debugu( 1, "NEWNEWNEWNEW SW_CMD: switch_id: %d, ACTION: %d, delay: %d", switch_id, ACTION, delay );
  ++( cmds->switch_idx );
  cmds->switch_cmds[cmds->switch_idx].switch_id = switch_id;
  cmds->switch_cmds[cmds->switch_idx].switch_action = ACTION;
  cmds->switch_cmds[cmds->switch_idx].switch_delay= delay;
}


inline void compute_next_command( train_state_t *train, rail_cmds_t* cmds ) {
  assertu( 1, cmds->train_id == NONE && cmds->switch_idx == NONE );
  track_node_t *track_graph = train->track_graph;
  int src_id = train->prev_sensor_id;
  int prev_sensor_id = train->prev_sensor_id;
  int second_sensor_id = NONE;
  int stop_dist = get_cur_stopping_distance( train );
  int train_len_ahead = get_len_train_ahead( train );
  int train_len_behind = get_len_train_behind( train );
  int cur_node_id = NONE;
  int action = NONE;
  int safe_branch_dist = safe_distance_to_branch( train );//train->speeds[train->cur_speed].safe_branch_distance;  

  debugu( 1, "src_id: %d, dest_id: %d, compute_next_command: dest_path_cur_idx: %d, total_dest_total_steps: %d", src_id, train->dest_path[train->dest_path_cur_idx], train->dest_path_cur_idx, train->dest_total_steps );
  debugu( 4,  "safe_branch_dist: %d", safe_branch_dist );
  assertum( 1, ( train->cur_speed >= 8 && train->cur_speed <= 14 ) || ( train->cur_speed >= 23 || train->cur_speed <= 29 ), "cur_speed: %d", train->cur_speed );

  for( ; train->dest_path_cur_idx < train->dest_total_steps; ++( train->dest_path_cur_idx )) {
    cur_node_id = train->dest_path[train->dest_path_cur_idx];

    /* update prev_sensor iff cur_sensor is the sensor immediately after src */
    if( track_graph[cur_node_id].type == NODE_SENSOR && second_sensor_id == -1 ) {
      debugu( 4,  "NODE SENSOR: %d, %s", cur_node_id, track_graph[cur_node_id].name );
      second_sensor_id = cur_node_id;
      prev_sensor_id = second_sensor_id;
    } 

    /* if the node is the end of the route, and if there is no other tarin command has been issued */
    if( train->dest_path_cur_idx == train->dest_total_steps - 1 && train->cur_speed != 0 && cmds->train_id == NONE ) {
      debugu( 4,  "END OF ROUTE: %d, %s", cur_node_id, track_graph[cur_node_id].name );
      /* get dist between sensor and dest */  
      int sensor2dest_dist = train->all_dist[cur_node_id] - train->all_dist[prev_sensor_id] + train->mm_past_dest; 
      /* if we are the last sensor or the dest and second sensor is too close */
      debugu( 4,  "prev_sensor_id: %d, src_id: %d, second_sensor_id: %d, stop_dist: %d, sensor2dest_dist: %d",
              prev_sensor_id, src_id, second_sensor_id, stop_dist, sensor2dest_dist );
      if( prev_sensor_id == src_id || ( prev_sensor_id == second_sensor_id && stop_dist > sensor2dest_dist )) {
        debugu( 4,  "computing stop command" );
        int src2dest_dist = train->all_dist[cur_node_id] + train->mm_past_dest;
        //int cur2dest_dist = src2dest_dist - train->mm_past_landmark / 10 - train_len_ahead;
        //int stop_delay_time = src2dest_dist > 0 ? get_delay_time_to_stop( train, cur2dest_dist ) : 0;
        int stop_delay_time = ((( src2dest_dist - stop_dist - train_len_ahead > 0 ) && train->cur_vel ) > 0 ? 
                          (( src2dest_dist - stop_dist - train_len_ahead ) * 10000 ) / train->cur_vel : 0 );
        debugu( 4,  "( %d - %d - %d) * 10000 / %d ", src2dest_dist, stop_dist, train_len_ahead, train->cur_vel );
        pack_train_cmd( cmds, train->train_id, TR_STOP, stop_delay_time );
        /* clear train destination */
        //train->dest_id = NONE;
      }
    }
    /* check reverse before branch, but handle reverse on branch separately, 
     * we should only handle one reverse at a time 
     */
    debugu( 1, "before reverse, train_id: %d", cmds->train_id );
    if(( train->dest_path_cur_idx - 1 ) >= 0 && cmds->train_id == NONE && // have previous node and nothing issued 
         track_graph[cur_node_id].reverse == &track_graph[train->dest_path[train->dest_path_cur_idx-1]] && // reverse 
         cmds->train_action != TR_REVERSE ) { // train is not currently reversing
      assertu( 1, cmds->train_action == NONE );
      debugu( 1,  "INSIDE REVERSE: cur_node: %d, node_name: %s", cur_node_id, track_graph[cur_node_id].name );
      assertu( 1, train->dest_path_cur_idx - 1 >= 0 && train->dest_path_cur_idx < train->dest_total_steps );
      
      int sensor2reverse_dist = train->all_dist[cur_node_id] - train->all_dist[prev_sensor_id];
      /* issue reverse command only if the train is standing at the reverse sensor */
      if( &(track_graph[train->dest_path[train->dest_path_cur_idx-1]]) - track_graph == src_id ) {
        debugu( 1, "calling pack_train_cmd on normal reverse" );
        pack_train_cmd( cmds, train->train_id, TR_REVERSE, 0 ); 
      }
      /* deal with reverse on branch */
      else if( track_graph[cur_node_id].type == NODE_BRANCH && ( prev_sensor_id == src_id || 
             ( prev_sensor_id  == second_sensor_id && stop_dist > sensor2reverse_dist + train_len_behind ))) {
        assertu( 1, track_graph[train->dest_path[train->dest_path_cur_idx-1]].type == NODE_MERGE );
        int src2reverse_dist = train->all_dist[cur_node_id];
        //int cur2dest_dist = src2reverse_dist + train_len_behind - train->mm_past_landmark / 10;
        //int reverse_delay_time = cur2dest_dist > 0 ? get_delay_time_to_stop( train, cur2dest_dist ) : 0; 
        int reverse_delay_time = ((( src2reverse_dist + train_len_behind - stop_dist > 0 ) && train->cur_vel > 0 ) ? 
          (( src2reverse_dist + train_len_behind - stop_dist ) * 10000 ) / train->cur_vel : 0 );// FIXME delay too short 
        debugu( 1, "calling pack_train_cmd on branch reverse" );
        pack_train_cmd( cmds, train->train_id, TR_REVERSE, reverse_delay_time );
      }
    }
    /* finally, branching case, only if we actually have a destination after the branch node */
    if( track_graph[cur_node_id].type == NODE_BRANCH && train->dest_path_cur_idx + 1 < train->dest_total_steps ) {
      debugu( 1,  "BRANCH: %d: %s, node after branch: %d: %s", cur_node_id, track_graph[cur_node_id].name,
          train->dest_path[train->dest_path_cur_idx+1], track_graph[train->dest_path[train->dest_path_cur_idx+1]].name );

      /* get dist between sensor and branch*/  
      int sensor2branch_dist = train->all_dist[cur_node_id] - train->all_dist[prev_sensor_id]; 
      int src2branch_dist = train->all_dist[cur_node_id];
      int branch_delay_time = train->cur_vel > 0 ? \
          (((( src2branch_dist - safe_branch_dist ) * 10000 ) / train->cur_vel - SW_TIME / 10 ) > 0 ? \
          (( src2branch_dist - safe_branch_dist ) * 10000 ) / train->cur_vel - SW_TIME / 10 : 0 ) : 0;
      debugu( 4,  "( %d - %d ) * 1000 / %d - %d", src2branch_dist, safe_branch_dist, train->cur_vel, SW_TIME/10 );

      /* issue switch commands */
      debugu( 4, "prev_sensor_id: %d, src_id: %d, sensor2branch_dist: %d, safe_branch_dist: %d",
          prev_sensor_id, src_id, sensor2branch_dist, safe_branch_dist ); 
      if(( prev_sensor_id == src_id && sensor2branch_dist >= safe_branch_dist) ||
         ( prev_sensor_id == second_sensor_id && sensor2branch_dist < safe_branch_dist )) {
        assertum( 1, track_graph[cur_node_id].num > 0 || track_graph[cur_node_id].num < 19 || 
            track_graph[cur_node_id].num > 152 || track_graph[cur_node_id].num < 157, "num: %d", track_graph[cur_node_id].num );
        assertu( 1, train->dest_path_cur_idx + 1 < train->dest_total_steps );
        assertu( 1, (( track_graph[cur_node_id].edge[DIR_STRAIGHT].dest == &track_graph[train->dest_path[train->dest_path_cur_idx+1]] ) || ( track_graph[cur_node_id].edge[DIR_CURVED].dest == &track_graph[train->dest_path[train->dest_path_cur_idx+1]] )));

        int switch_id = track_graph[cur_node_id].num;
        action = ( track_graph[cur_node_id].edge[DIR_STRAIGHT].dest == &track_graph[train->dest_path[train->dest_path_cur_idx+1]] ) ? SW_STRAIGHT : SW_CURVED;
        debugu( 4, "calling pack_swtich_cmd with switch_id: %d", switch_id );
        pack_switch_cmd( cmds, switch_id, action, branch_delay_time );
      }
      else {
        debugu( 4,  "branch to be handled at next sensor hit" );   
      }
    } 
    else {
      debugu( 4,  "DON'T CARE NODE: %d, %s", cur_node_id, track_graph[cur_node_id].name );
    }
  }

}

//int predict_next_sensor_dynamic(  );
void request_next_command( train_state_t* train, rail_cmds_t* cmds ) {
  // for now, we only get called if there is a destination, but we should also make decisions for avoiding colissions
  assertu( 1, train->dest_id != NONE ); 
  debugu( 1,  "request_next_command with src_id: %d and dest_id: %d", train->prev_sensor_id, train->dest_id );
  track_node_t* track_graph = train->track_graph;
  int src_id = train->prev_sensor_id;
  int dest_id = train->dest_id;

  /* currently we only run dijkstra on sensor hit, so src_id should be  a sensor */
  assertu( 1, track_graph[src_id].type == NODE_SENSOR );
  assertu( 1, track_graph && cmds && src_id >= 0 && src_id < TRACK_MAX && dest_id >= 0 && dest_id < TRACK_MAX );

  /* calculate shortest path by dijkstra or train's most recent memory */
  get_shortest_path( train );
  //Printf( COM2, "get_next_command with: train_id: %d, src_id: %d, dest_id: %d, train->cur_speed: %d, train->cur_vel: %d, stop_dist: %d, safe_branch_dist: %d\n\r", train_id, src_id, dest_id, train->cur_speed, train->cur_vel, stop_dist, safe_branch_dist );

  compute_next_command( train, cmds );
  //print_cmds( cmds );
}


//TODO: put below into a separate file "rail_control_helper"
inline void init_node( min_heap_node_t * node, int id, int dist ) {
  assertu( 1, node || id >= 0 || id < NODE_MAX );

  node->id = id;
  node->dist = dist;
}

void init_min_heap( min_heap_t * min_heap, int src_id, int * node_id2idx, min_heap_node_t * nodes ) {
  assertu( 1, min_heap );

  min_heap->size = NODE_MAX;
  min_heap->capacity = NODE_MAX;
  min_heap->node_id2idx = node_id2idx;
  min_heap->nodes = nodes;

  int i;
  for( i = 0; i < min_heap->capacity; ++i ) {
    min_heap->node_id2idx[i] = i;
    init_node( &( min_heap->nodes[i]), i, INT_MAX );
  }

  decrease_dist( min_heap, src_id, 0 );

}

inline void swap_node( min_heap_node_t * node_a, min_heap_node_t * node_b ){
  assertu( 1, node_a && node_b );

  int id_tmp, dist_tmp;
  id_tmp = node_a->id;
  dist_tmp = node_a->dist;
  node_a->id = node_b->id;
  node_a->dist = node_b->dist;
  node_b->id = id_tmp;
  node_b->dist = dist_tmp;
}

void make_min_heap( min_heap_t * min_heap, int idx ) {
  assertu( 1, min_heap );
  assertu( 1, idx >= 0 && idx < min_heap->size );

  int smallest, left, rite;
  smallest = idx;
  left = 2 * idx + 1;
  rite = 2 * idx + 2;

  if( left < min_heap->size && // make sure left child is in range
      min_heap->nodes[left].dist < min_heap->nodes[smallest].dist ) 
    smallest = left;

  if( rite < min_heap->size && // make sure rite child is in range
      min_heap->nodes[rite].dist < min_heap->nodes[smallest].dist ) 
    smallest = rite;

  /* if idx's has a child that's smaller, we bubble up the smallest child */
  if( smallest != idx ) {
    min_heap_node_t * parent_node   = &(min_heap->nodes[idx]);
    min_heap_node_t * smallest_node = &(min_heap->nodes[smallest]);

    /* update the id2idx array, to reflect the below swapping */
    assertu( 1, min_heap->node_id2idx[parent_node->id] == idx ); 
    if( min_heap->node_id2idx[smallest_node->id] != smallest ) {
      //debugu( 1, "smallest_node->idx: %d, smallest: %d", min_heap->node_id2idx[smallest_node->id], smallest );
    }
    assertu( 1, min_heap->node_id2idx[smallest_node->id] == smallest );
    min_heap->node_id2idx[parent_node->id] = smallest;
    min_heap->node_id2idx[smallest_node->id] = idx;

    /* swap parent and child in the nodes array */
    swap_node( parent_node, smallest_node );

    /* recursively call heapify on the child idx */
    make_min_heap( min_heap, smallest );
  }
}

inline bool heap_empty( min_heap_t * min_heap ) {
  return min_heap->size == 0;
}

min_heap_node_t * extract_min( min_heap_t * min_heap ) {
  if( heap_empty( min_heap ))
    return NULL;

  int last_idx = min_heap->size - 1;
  min_heap_node_t * root = &(min_heap->nodes[0]);
  min_heap_node_t * last = &(min_heap->nodes[last_idx]);

  /* update the id2idx map */
  min_heap->node_id2idx[last->id] = 0;
  min_heap->node_id2idx[root->id] = last_idx;

  /* swap the last and root */
  swap_node( root, last );

  /* decrease the heap size */
  --( min_heap->size );
  
  /* heapify the root */
  if( min_heap->size > 0 )
    make_min_heap( min_heap, 0 );

  assertum( 1, last->dist < INT_MAX, "if this fails, make sure you set 140 nodes to trackB and 144 to trackA" );
  //TODO: remove below testing code
  if( last->dist >= INT_MAX ) {
    debugu( 1,  "extracted: id: %d, dist: %d", last->id, last->dist );
    print_min_heap( min_heap );
    FOREVER;
  }

  return last;  
}

void decrease_dist( min_heap_t * min_heap, int id, int dist ) {
  //debugu( 1,  "decrease_dist: id: %d, dist: %d", id, dist );
  assertu( 1, min_heap );
  
  /* find the node with the vertex id */
  int idx = min_heap->node_id2idx[id];
  assertu( 1, idx >= 0 );
  assertum( 1, idx <= min_heap->size, "idx: %d, size: %d", idx, min_heap->size );
  assertum( 1, dist < min_heap->nodes[idx].dist, "old dist: %d, new_dist: %d", min_heap->nodes[idx].dist, dist );
  min_heap->nodes[idx].dist = dist;

  /* bubble up the node with the updated distance */
  int parent_idx = ( idx - 1 ) / 2;
  //debugu( 1,  "idx dist: %d, idx/2 dist: %d", min_heap->nodes[idx].dist, min_heap->nodes[parent_idx].dist );
  while( idx != 0 && min_heap->nodes[idx].dist < min_heap->nodes[parent_idx].dist ) {
    //debugu( 1, " heapify idx: %d", idx );
    /* swap the idx */
    min_heap->node_id2idx[min_heap->nodes[parent_idx].id] = idx;
    min_heap->node_id2idx[min_heap->nodes[idx].id] = parent_idx;

    /* swap this node with its parent */
    swap_node( &( min_heap->nodes[idx] ), &( min_heap->nodes[parent_idx] ));
    
    idx = parent_idx;
    parent_idx = ( idx - 1 ) / 2;
  }
  //debugu( 1,  "idx: %d, parent_idx: %d, idx_dist: %d, parent_dist: %d", idx,
  //    parent_idx, min_heap->nodes[idx].dist, min_heap->nodes[parent_idx].dist );
}

inline bool heap_find( min_heap_t * min_heap, int id ) {
  return min_heap->node_id2idx[id] < min_heap->size;
}

inline void print_min_heap( min_heap_t * min_heap ) {
  assertu( 1, min_heap );
  //Printf( COM2, "size: %d\n\r", min_heap->size );
  int idx;
  for( idx = 0; idx < min_heap->size; ++idx ) {
    //Printf( COM2, "id: %d, dist: %d", min_heap->nodes[idx].id, min_heap->nodes[idx].dist );
  }
  //Printf( COM2, "\n\r" );
}

void dijkstra( struct _track_node_* track_graph, int src_id, int* path, int* dist, int* step ) {
  assertu( 1, track_graph || src_id >= 0 || src_id < NODE_MAX )
  
  /* initialize the heap */
  min_heap_t min_heap;
  int node_id2idx[NODE_MAX];
  min_heap_node_t nodes[NODE_MAX];
  init_min_heap( &min_heap, src_id, node_id2idx, nodes );

  /* initialize all distance to INT_MAX, all path to invalid */
  int i;
  for( i = 0; i < NODE_MAX; ++i ) {
    dist[i] = INT_MAX;
    path[i] = -1;
    step[i] = 0;
  }

  /* make the distance of the source to 0, so it's first picked */
  dist[src_id] = 0;

  /* in our case all vertices are connected, so loop until the heap is empty */
  //TODO: think about replacing edge weight with time
  //TODO: if use time, reverse edge weight should be dynamically calculated
  while( !heap_empty( &min_heap )) {
    assertu( 1, min_heap.size );
    min_heap_node_t * heap_node = extract_min( &min_heap );
    assertu( 1, heap_node );
    //debugu( 1,  "min node_id: %d", heap_node->id );
    
    int track_id = heap_node ->id;
    track_node_t * track_node = &(track_graph[track_id]);
    
    /* helper functions to calculate dist to all neighbors, 
     * NODE_ENTER, NODE_SENEOR, NODE_MERGE have two neighbors: straight & reverse;
     * NODE_BRANCH has three neighbors: straight, curved and reverse;
     * NODE_EXIT has one neighbor: reverse 
     */
    int track_nbr_id, test_dist, test_step;
    #define update_info( ) \
      if( heap_find( &min_heap, track_nbr_id ) && \
          test_dist < dist[track_nbr_id] ) { \
        dist[track_nbr_id] = test_dist; \
        path[track_nbr_id] = track_id; \
        step[track_nbr_id] = test_step; \
        decrease_dist( &min_heap, track_nbr_id, test_dist ); \
      }

    inline void __attribute__((always_inline)) \
    update_forward( int direction ) {
      track_nbr_id = track_node->edge[direction].dest - track_graph;
      assertu( 1, track_nbr_id >= 0 );
      test_dist = dist[track_id] + track_node->edge[direction].dist;
      assertum( 1, dist[track_nbr_id] == min_heap.nodes[min_heap.node_id2idx[track_nbr_id]].dist, "dist: %d, heap_dist: %d\n\r\n\r\n\r", dist[track_nbr_id], min_heap.nodes[min_heap.node_id2idx[track_nbr_id]].dist );
      test_step = step[track_id] + 1;
      update_info( );
    }

    inline void __attribute__((always_inline)) \
    update_backward( ) {
      track_nbr_id = track_node->reverse - track_graph;
      assertu( 1, track_nbr_id >= 0 );
      test_dist = dist[track_id];// + 1000000;//dist[track_id] // FIXME: this is so that we do not have reverse case
      assertum( 1, dist[track_nbr_id] == min_heap.nodes[min_heap.node_id2idx[track_nbr_id]].dist, "dist: %d, heap_dist: %d", dist[track_nbr_id], min_heap.nodes[min_heap.node_id2idx[track_nbr_id]].dist );
      test_step = step[track_id] + 1;
      update_info( );
    }

    switch( track_node->type ) {
      case NODE_ENTER:
        /* forward */
        update_forward( DIR_AHEAD ); 
        /* backward */
        update_backward( );
        break;
      case NODE_SENSOR:
        /* forward */
        update_forward( DIR_AHEAD ); 
        /* backward */
        update_backward( );
        break;
      case NODE_MERGE:
        /* forward */
        update_forward( DIR_AHEAD ); 
        /* backward */
        update_backward( );
        break;
      case NODE_BRANCH:
        /* forward */
        update_forward( DIR_AHEAD );
        update_forward( DIR_CURVED );
        /* backward */
        update_backward( );
        break;
      case NODE_EXIT:
        /* backward */
        update_backward( );
        break;
      default:
        assertu( 1, false );
        break;
    }

    assertum( 1,dist[track_id]==min_heap.nodes[min_heap.node_id2idx[track_id]].dist, 
        "unmatched track_id: %d, dist_arr: %d, heap_dist: %d", track_id, 
        dist[track_id], min_heap.nodes[min_heap.node_id2idx[track_id]].dist );
  }
}


void extract_shortest_path( int* all_path, int* all_step, int src_id, int dest_id, int* dest_path ) {
  int steps = all_step[dest_id] - 1;
  int i = NODE_MAX - 1;
  while( i > steps ) {
    dest_path[i] = NONE;
    --i;
  }
  while( steps >= 0 ) {
    dest_path[steps] = dest_id;
    dest_id = all_path[dest_id];
    --(steps);
  }
  assertum( 1, steps == -1, "steps: %d", steps );
  
}

void print_shortest_path( track_node_t* track_graph, int* all_path, int* all_step, int src_id, int dest_id, int* dest_path ) {
  int tmp_id = dest_id;
  int steps = all_step[tmp_id] - 1;
  while( steps >= 0 ) {
    dest_path[steps] = tmp_id;

    if( tmp_id == src_id )
      break;

    tmp_id = all_path[tmp_id];
    --(steps);
  }
  assertu( 1, steps == -1 );

  steps = all_step[dest_id];
  int i;
#if( BWAIT == 1 )
  bwprintf( COM2, "SHORTEST PATH.................: " ); 
  bwprintf( COM2, "%d steps from %s to %s:\t%s", steps, track_graph[src_id].name,
      track_graph[dest_id].name, track_graph[src_id].name );
  for( i = 0; i < steps; ++i ) {
    bwprintf( COM2, "->%s", track_graph[dest_path[i]].name );
  }
  bwprintf( COM2, "\r\n" );
#else 
  Printf( COM2, "SHORTEST PATH: " ); 
  Printf( COM2, "%d steps from %s to %s:\t%s", steps, track_graph[src_id].name,
      track_graph[dest_id].name, track_graph[src_id].name );
  for( i = 0; i < steps; ++i ) {
    Printf( COM2, "->%s", track_graph[dest_path[i]].name );
  }
  Printf( COM2, "\r\n" );

#endif 
}

void print_cmds( struct _rail_cmds_ * cmds ) {
#if( BWAIT == 1 )
  bwprintf( COM2, "PRINTING NEW COMMANDS .............. \n\r" );
  bwprintf( COM2, "train_id: %d, train_action: %d, tarin_delay: %d\n\r", cmds->train_id, cmds->train_action, cmds->train_delay );
  int i;
  for( i = 0; i < SW_CMD_MAX; ++i ) {
    bwprintf( COM2, "sw_id: %d, sw_action: %d, sw_delay: %d\n\r", 
        cmds->switch_cmds[i].switch_id, cmds->switch_cmds[i].switch_action, cmds->switch_cmds[i].switch_delay );
  }
#else 
  Printf( COM2, "PRINTING NEW COMMANDS .............. \n\r" );
  Printf( COM2, "train_id: %d, train_action: %d, tarin_delay: %d\n\r", cmds->train_id, cmds->train_action, cmds->train_delay );
  int i;
  for( i = 0; i < SW_CMD_MAX; ++i ) {
    Printf( COM2, "sw_id: %d, sw_action: %d, sw_delay: %d\n\r", 
        cmds->switch_cmds[i].switch_id, cmds->switch_cmds[i].switch_action, cmds->switch_cmds[i].switch_delay );
  }
#endif
}
