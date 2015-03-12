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
  cmds->sw_count = cmds->train_id = cmds->train_action = cmds->train_delay = cmds->train_speed = \
  cmds->switch_id0 = cmds->switch_action0= cmds->switch_delay0 = \
  cmds->switch_id1 = cmds->switch_action1= cmds->switch_delay1 = \
  cmds->switch_id2 = cmds->switch_action2= cmds->switch_delay2 = \
  cmds->switch_id3 = cmds->switch_action3 = cmds->switch_delay3 = NONE;

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
//  //assert( 1, train_state
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
//  //  assertm( 1, false, "calling predict_next_sensor_dynamic in unnecessary condition" );
//  //}
//
//}

void predict_next_sensor_dynamic( train_state_t* train_state ) {
  assert( 1, train_state->cur_speed == 0 || train_state->state == REVERSING );
  track_node_t* cur_node = &( train_state->track_graph[train_state->prev_sensor_id] );
  assert( 1, cur_node );
  assert( 1, cur_node->type == NODE_SENSOR );
  int next_sensor_id = NONE;
  int next_sensor_dist = cur_node->edge[DIR_AHEAD].dist;
  int branch_ind;
  cur_node = cur_node->edge[DIR_AHEAD].dest;
  //int stop_dist = get_cur_stopping_distance( train_state );
  int stop_dist = get_cur_stopping_distance( train_state );
  while( next_sensor_id < 0 && next_sensor_dist <= stop_dist ) {
    debug("whileloop, next_sensor_dist: %d, stop_dist: %d", next_sensor_dist, stop_dist ) ;
    if( cur_node->type == NODE_SENSOR ) {
      debug( "1" );
      next_sensor_id = cur_node->num;
      assert( 1, next_sensor_id >= 0 );
    }
    else if( cur_node->type == NODE_BRANCH ) {
      debug( "2" );
      branch_ind = cur_node->num;
      if( branch_ind > 152 ) {
        branch_ind -= 134;
      }
      next_sensor_dist += cur_node->edge[train_state->switch_states[branch_ind]].dist;
      cur_node = cur_node->edge[train_state->switch_states[branch_ind]].dest;
    }
    else if( cur_node->type == NODE_EXIT ) {
      debug( "3" );
      break;
    }
    else {
      debug( "4" );
      next_sensor_dist += cur_node->edge[DIR_AHEAD].dist;
      cur_node = cur_node->edge[DIR_AHEAD].dest;
    }
  }

  if( next_sensor_id == NONE && train_state->state == REVERSING ) {
    debug( "5" );
    train_state->next_sensor_id = train_state->track_graph[train_state->prev_sensor_id].reverse - train_state->track_graph;
    train_state->dist_to_next_sensor = (stop_dist * 2) - train_state->pickup_len > 0 ? (stop_dist * 2) - train_state->pickup_len : 0;
  }

}

void predict_next_sensor_static( train_state_t *train_state ) {
  track_node_t* cur_node = &(train_state->track_graph[train_state->prev_sensor_id]);
  assert( 1, cur_node && cur_node->type == NODE_SENSOR );
  int next_sensor_id = NONE; // might be an Exit node
  int ret_node_dist = cur_node->edge[DIR_AHEAD].dist;
  int branch_ind;
  cur_node = cur_node->edge[DIR_AHEAD].dest;

  while( next_sensor_id < 0 ) {
    //Printf( COM2, "cur_node->type: %d, cur_node->num: %d\r\n", cur_node->type, cur_node->num );
    if( cur_node->type == NODE_SENSOR ) {
      //Printf( COM2, "sensor node\r\n" );
      next_sensor_id = cur_node->num; 
      assert( 1, next_sensor_id >= 0 );
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
  
  debug( "dist_to_next_sensor: %d", ret_node_dist );
  debug( "next_sensor_id: %d", next_sensor_id );
  train_state->dist_to_next_sensor = ret_node_dist;
  train_state->next_sensor_id = next_sensor_id; 
}

void predict_next_fallback_sensors_static( train_state_t *train ) {
  track_node_t* cur_node = &(train->track_graph[train->prev_sensor_id]);
  int expected_sensor_num = train->next_sensor_id;
  if( (cur_node->reverse)->num == expected_sensor_num ) {
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
  assert( 1, cur_node && cur_node->type == NODE_SENSOR );
  int branch_ind;
  int fallback_idx = 0;
  int i;
  //int cur_node_id;
  for( i = 0; i < NUM_FALLBACK; ++i ) {
    train->fallback_sensors[i] = -1;
  }
  declare_ring_queue( track_node_t*, node_id, 10 );
  node_id_push_back( cur_node->edge[DIR_AHEAD].dest );

  while( !node_id_empty( ) ) {
    cur_node = node_id_pop_front( );
    //assert( 1, node_id_top_front( ) != cur_node_id );
    if( cur_node->type == NODE_SENSOR ) {
      if( cur_node->num == expected_sensor_num ) {
        saw_expected_sensor = 1;
        node_id_push_back( cur_node->edge[DIR_AHEAD].dest );
      } else {
        train->fallback_sensors[fallback_idx++] = cur_node->num;
      }
    }
    else if( cur_node->type == NODE_BRANCH ) {
      if( saw_expected_sensor ) {
        branch_ind = cur_node->num;
        if( branch_ind > 152 ) {
          branch_ind -= 134;
        }
        node_id_push_back( cur_node->edge[train->switch_states[branch_ind]].dest );
        //assert( 1, node_id_top_back( ) == (cur_node->edge[train->switch_states[branch_ind]].dest)->num );
      } else {
        node_id_push_back( cur_node->edge[DIR_STRAIGHT].dest );
        //assert( 1, node_id_top_back( ) == (cur_node->edge[DIR_STRAIGHT].dest)->num );
        node_id_push_back( cur_node->edge[DIR_CURVED].dest );
        //assert( 1, node_id_top_back( ) == (cur_node->edge[DIR_CURVED].dest)->num );
      }
    } else if( cur_node->type == NODE_EXIT ) {
      continue;
    } else { /* any other node */
      node_id_push_back( cur_node->edge[DIR_AHEAD].dest );
      //assert( 1, node_id_top_back( ) == (cur_node->edge[DIR_AHEAD].dest)->num );
    }
  }
}

//int predict_next_sensor_dynamic(  );
void get_next_command( train_state_t* train, rail_cmds_t* cmds ) {
  debug( "get_next_command with cur_node_id: %d and dest_id: %d", train->prev_sensor_id, train->dest_id );
  track_node_t* track_graph = train->track_graph;
  int train_id = train->train_id;
  int src_id = train->prev_sensor_id;
  int dest_id = train->dest_id;
  int speed_idx = train->cur_speed;
  assertm( 1, ( speed_idx >= 8 && speed_idx <= 14 ) || ( speed_idx >= 23 || speed_idx <= 29 ), "cur_speed: %d", speed_idx );
  int train_velocity = train->cur_vel; 
  //int stop_time = train->speeds[speed_idx].stopping_time;
  int stop_dist = get_cur_stopping_distance( train );
  int safe_branch_dist = safe_distance_to_branch( train ); 
  debug( "safe_branch_dist: %d", safe_branch_dist );

  //DEBUG
  //Printf( COM2, "get_next_command with: train_id: %d, src_id: %d, dest_id: %d, speed_idx: %d, train_velocity: %d, stop_dist: %d, safe_branch_dist: %d\n\r", train_id, src_id, dest_id, speed_idx, train_velocity, stop_dist, safe_branch_dist );

  assert( 1, track_graph && cmds && src_id >= 0 && src_id < TRACK_MAX && dest_id >= 0 && dest_id < TRACK_MAX );
  /* currently we only run dijkstra on sensor hit, so src_id should be  a sensor */
  assert( 1, track_graph[src_id].type == NODE_SENSOR );

  /* run dijkstra on the src and dest */
  int all_path[NODE_MAX], all_dist[NODE_MAX], all_step[NODE_MAX];
  dijkstra( track_graph, src_id, all_path, all_dist, all_step );
  debug( "dist from src to dest: %d", all_dist[dest_id] );
  
  /* get shortest path for our destination */
  int dest_path[all_step[dest_id]];
  get_shortest_path( all_path, all_step, src_id, dest_id, dest_path );

  int i;
  int prev_sensor_id = src_id;
  int second_sensor_id = -1;
  int steps_to_dest = all_step[dest_id];
  int cur_node_id;
  int switch_count = 0;
  int action;


  print_shortest_path( track_graph, all_path, all_step, src_id, dest_id, dest_path);
  debug( "steps_to_dest: %d", steps_to_dest );
  for( i = 0; i < steps_to_dest; ++i ) {
    cur_node_id = dest_path[i];

    /* update prev_sensor iff cur_sensor is the sensor immediately after src */
    if( track_graph[cur_node_id].type == NODE_SENSOR && second_sensor_id == -1 ) {
      debug( "NODE SENSOR: %d, %s", cur_node_id, track_graph[cur_node_id].name );
      second_sensor_id = cur_node_id;
      prev_sensor_id = second_sensor_id;
    } 

    /* if the node is the end of the route */
    if( i == steps_to_dest - 1 && train->cur_speed != 0 ) {
      debug( "END OF ROUTE: %d, %s", cur_node_id, track_graph[cur_node_id].name );
      /* get dist between sensor and dest */  
      int sensor2dest_dist = all_dist[cur_node_id] - all_dist[prev_sensor_id] + train->mm_past_dest; 
      /* if we are the last sensor or the dest and second sensor is too close */
      debug( "prev_sensor_id: %d, src_id: %d, second_sensor_id: %d, stop_dist: %d, sensor2dest_dist: %d",
              prev_sensor_id, src_id, second_sensor_id, stop_dist, sensor2dest_dist );
      if( prev_sensor_id == src_id || 
         ( prev_sensor_id == second_sensor_id && stop_dist > sensor2dest_dist )) {
        debug( "setting stop command" );
        int src2dest_dist = all_dist[cur_node_id] + train->mm_past_dest;
        
        cmds->train_id = train_id;
        cmds->train_action = TR_STOP;
        cmds->train_delay = (( src2dest_dist - stop_dist - get_len_train_ahead( train )) > 0 ) ? (( src2dest_dist - stop_dist - get_len_train_ahead( train ) ) * 10000 ) / train_velocity : 0;
        debug( "( %d - %d ) * 10000 / %d ", src2dest_dist, stop_dist, train_velocity );

        /* clear train destination */
        //DEBUG
        //train->dest_id = NONE;
        //Printf( COM2, "CLEARING TRAIN DESTINATION: %d\n\r", train->dest_id );
      }
    }
    /* check reverse before branch, but handle reverse on branch separately, 
     * we should only handle one reverse at a time 
     */
    if(( i - 1 ) >= 0 && 
        ( track_graph[cur_node_id].reverse == &track_graph[dest_path[i-1]] && cmds->train_action != TR_REVERSE )) {
      assert( 1, cmds->train_action == NONE );
      debug( "REVERSE: %d, %s", cur_node_id, track_graph[cur_node_id].name );
      assert( 1, i - 1 >= 0 && i < steps_to_dest );
      int sensor2reverse_dist = all_dist[cur_node_id] - all_dist[prev_sensor_id];

      /* issue reverse command only if the train is standing at the reverse sensor */
      if( &(track_graph[dest_path[i-1]]) - track_graph == src_id ) {
        cmds->train_id = train_id;
        cmds->train_action = TR_REVERSE;
        cmds->train_delay = 0;//all_dist[cur_node_id] - all_dist[src_id];
      }

      /* deal with reverse on branch */
      else if( track_graph[cur_node_id].type == NODE_BRANCH && ( prev_sensor_id == src_id || 
            ( prev_sensor_id  == second_sensor_id && stop_dist > sensor2reverse_dist ))) {
        assert( 1, track_graph[dest_path[i-1]].type == NODE_MERGE );
        int src2reverse_dist = all_dist[cur_node_id];

        cmds->train_id = train_id;
        cmds->train_action = TR_REVERSE;
        cmds->train_delay = (( src2reverse_dist - stop_dist ) > 0 ) ? \
          (( src2reverse_dist ) * 10000 ) / train_velocity : 0;// FIXME: should be src2reverse_dist - stop_dist
      }
      
    }
    /* finally, branching case, and if we actually specify a destination after the branch node */
    if( track_graph[cur_node_id].type == NODE_BRANCH && i + 1 < steps_to_dest ) {
      debug( "BRANCH: %d, %s", cur_node_id, track_graph[cur_node_id].name );
      action = -1;

      #define set_switch( _cmds, _switch_count, _switch_id, _action, _delay ) \
        ++(_cmds->sw_count); \
        _cmds->switch_id##_switch_count = _switch_id; \
        _cmds->switch_action##_switch_count = _action; \
        _cmds->switch_delay##_switch_count = _delay; 
      /* get dist between sensor and branch*/  
      int sensor2branch_dist = all_dist[cur_node_id] - all_dist[prev_sensor_id]; 
      int src2branch_dist = all_dist[cur_node_id];
      int branch_delay_time = 0;//((( src2branch_dist - safe_branch_dist ) * 10000 ) / train_velocity - SW_TIME/10 ) > 0 ?
                        //(( src2branch_dist - safe_branch_dist ) * 10000 ) / train_velocity - SW_TIME/10  : 0;
      debug( "( %d - %d ) * 1000 / %d - %d", src2branch_dist, safe_branch_dist, train_velocity, SW_TIME );

      /* issue switch commands */
      if(( prev_sensor_id == src_id && sensor2branch_dist >= safe_branch_dist) ||
         ( prev_sensor_id == second_sensor_id && sensor2branch_dist < safe_branch_dist )) {
        //FIXME: this is faling, should handle 4 branchees
        assert( 1, switch_count < 4 );
        assertm( 1, track_graph[cur_node_id].num > 0 || 
                   track_graph[cur_node_id].num < 19 || 
                   track_graph[cur_node_id].num > 152 || 
                   track_graph[cur_node_id].num < 157, 
                   "num: %d", track_graph[cur_node_id].num );
        assert( 1, i + 1 < steps_to_dest );
        assert( 1, (( track_graph[cur_node_id].edge[DIR_STRAIGHT].dest == &track_graph[dest_path[i+1]] ) || 
                    ( track_graph[cur_node_id].edge[DIR_CURVED].dest == &track_graph[dest_path[i+1]] )));

        action = ( track_graph[cur_node_id].edge[DIR_STRAIGHT].dest == &track_graph[dest_path[i+1]] ) ? SW_STRAIGHT : SW_CURVED;

        int switch_id = track_graph[cur_node_id].num;
        CONVERT_SWITCH_ID( switch_id );
        if( switch_count == 0 ){
          set_switch( cmds, 0, switch_id, action, branch_delay_time );
        }
        else if( switch_count == 1 ) {
          set_switch( cmds, 1, switch_id, action, branch_delay_time );
        }
        else if( switch_count == 2 ) {
          set_switch( cmds, 2, switch_id, action, branch_delay_time );
        }
        else if( switch_count == 3 ) {
          set_switch( cmds, 3, switch_id, action, branch_delay_time );
        }
        else
          assertm( 1, false, "too many branches than we can handle, increase switch numbers" );
        
        ++switch_count;
      }
      else {
        debug( "branch to be handled at next sensor hit" );   
      }
    } 
    else {
      debug( "DON'T CARE NODE: %d, %s", cur_node_id, track_graph[cur_node_id].name );
    }
  }
}


//TODO: put below into a separate file "rail_control_helper"
inline void init_node( min_heap_node_t * node, int id, int dist ) {
  assert( 1, node || id >= 0 || id < NODE_MAX );

  node->id = id;
  node->dist = dist;
}

void init_min_heap( min_heap_t * min_heap, int src_id, int * node_id2idx, min_heap_node_t * nodes ) {
  assert( 1, min_heap );

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
  assert( 1, node_a && node_b );

  int id_tmp, dist_tmp;
  id_tmp = node_a->id;
  dist_tmp = node_a->dist;
  node_a->id = node_b->id;
  node_a->dist = node_b->dist;
  node_b->id = id_tmp;
  node_b->dist = dist_tmp;
}

void make_min_heap( min_heap_t * min_heap, int idx ) {
  assert( 1, min_heap );
  assert( 1, idx >= 0 && idx < min_heap->size );

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
    assert( 1, min_heap->node_id2idx[parent_node->id] == idx ); 
    if( min_heap->node_id2idx[smallest_node->id] != smallest ) {
      //debug("smallest_node->idx: %d, smallest: %d", min_heap->node_id2idx[smallest_node->id], smallest );
    }
    assert( 1, min_heap->node_id2idx[smallest_node->id] == smallest );
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

  assertm( 1, last->dist < INT_MAX, "if this fails, make sure you set 140 nodes to trackB and 144 to trackA" );
  //TODO: remove below testing code
  if( last->dist >= INT_MAX ) {
    debug( "extracted: id: %d, dist: %d", last->id, last->dist );
    print_min_heap( min_heap );
    FOREVER;
  }

  return last;  
}

void decrease_dist( min_heap_t * min_heap, int id, int dist ) {
  //debug( "decrease_dist: id: %d, dist: %d", id, dist );
  assert( 1, min_heap );
  
  /* find the node with the vertex id */
  int idx = min_heap->node_id2idx[id];
  assert( 1, idx >= 0 );
  assertm( 1, idx <= min_heap->size, "idx: %d, size: %d", idx, min_heap->size );
  assertm( 1, dist < min_heap->nodes[idx].dist, "old dist: %d, new_dist: %d", min_heap->nodes[idx].dist, dist );
  min_heap->nodes[idx].dist = dist;

  /* bubble up the node with the updated distance */
  int parent_idx = ( idx - 1 ) / 2;
  //debug( "idx dist: %d, idx/2 dist: %d", min_heap->nodes[idx].dist, min_heap->nodes[parent_idx].dist );
  while( idx != 0 && min_heap->nodes[idx].dist < min_heap->nodes[parent_idx].dist ) {
    //debug(" heapify idx: %d", idx );
    /* swap the idx */
    min_heap->node_id2idx[min_heap->nodes[parent_idx].id] = idx;
    min_heap->node_id2idx[min_heap->nodes[idx].id] = parent_idx;

    /* swap this node with its parent */
    swap_node( &( min_heap->nodes[idx] ), &( min_heap->nodes[parent_idx] ));
    
    idx = parent_idx;
    parent_idx = ( idx - 1 ) / 2;
  }
  //debug( "idx: %d, parent_idx: %d, idx_dist: %d, parent_dist: %d", idx,
  //    parent_idx, min_heap->nodes[idx].dist, min_heap->nodes[parent_idx].dist );
}

inline bool heap_find( min_heap_t * min_heap, int id ) {
  return min_heap->node_id2idx[id] < min_heap->size;
}

inline void print_min_heap( min_heap_t * min_heap ) {
  assert( 1, min_heap );
  //Printf( COM2, "size: %d\n\r", min_heap->size );
  int idx;
  for( idx = 0; idx < min_heap->size; ++idx ) {
    //Printf( COM2, "id: %d, dist: %d", min_heap->nodes[idx].id, min_heap->nodes[idx].dist );
  }
  //Printf( COM2, "\n\r" );
}

void dijkstra( struct _track_node_* track_graph, int src_id, int* path, int* dist, int* step ) {
  assert( 1, track_graph || src_id >= 0 || src_id < NODE_MAX )
  
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
    assert( 1, min_heap.size );
    min_heap_node_t * heap_node = extract_min( &min_heap );
    assert( 1, heap_node );
    //debug( "min node_id: %d", heap_node->id );
    
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
      assert( 1, track_nbr_id >= 0 );
      test_dist = dist[track_id] + track_node->edge[direction].dist;
      assertm( 1, dist[track_nbr_id] == min_heap.nodes[min_heap.node_id2idx[track_nbr_id]].dist, "dist: %d, heap_dist: %d", dist[track_nbr_id], min_heap.nodes[min_heap.node_id2idx[track_nbr_id]].dist );
      test_step = step[track_id] + 1;
      update_info( );
    }

    inline void __attribute__((always_inline)) \
    update_backward( ) {
      track_nbr_id = track_node->reverse - track_graph;
      assert( 1, track_nbr_id >= 0 );
      test_dist = dist[track_id];// + 1000000;//dist[track_id] // FIXME: this is so that we do not have reverse case
      assertm( 1, dist[track_nbr_id] == min_heap.nodes[min_heap.node_id2idx[track_nbr_id]].dist, "dist: %d, heap_dist: %d", dist[track_nbr_id], min_heap.nodes[min_heap.node_id2idx[track_nbr_id]].dist );
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
        assert( 1, false );
        break;
    }

    assertm( 1,dist[track_id]==min_heap.nodes[min_heap.node_id2idx[track_id]].dist, 
        "unmatched track_id: %d, dist_arr: %d, heap_dist: %d", track_id, 
        dist[track_id], min_heap.nodes[min_heap.node_id2idx[track_id]].dist );
  }
}


void get_shortest_path( int* all_path, int* all_step, int src_id, int dest_id, int* dest_path ) {
  int steps = all_step[dest_id] - 1;
  while( steps >= 0 ) {
    dest_path[steps] = dest_id;
    dest_id = all_path[dest_id];
    --(steps);
  }
  assertm( 1, steps == -1, "steps: %d", steps );
  
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
  assert( 1, steps == -1 );

  steps = all_step[dest_id];
  int i;
#if( DEBUG == 0 )
  //Printf( COM2, "SHORTEST PATH: " ); 
  //Printf( COM2, "%d steps from %s to %s:\t%s", steps, track_graph[src_id].name,
  //    track_graph[dest_id].name, track_graph[src_id].name );
  for( i = 0; i < steps; ++i ) {
    //Printf( COM2, "->%s", track_graph[dest_path[i]].name );
  }
  //Printf( COM2, "\r\n" );
#else 
  bwprintf( COM2, "SHORTEST PATH: " ); 
  bwprintf( COM2, "%d steps from %s to %s:\t%s", steps, track_graph[src_id].name,
      track_graph[dest_id].name, track_graph[src_id].name );
  for( i = 0; i < steps; ++i ) {
    bwprintf( COM2, "->%s", track_graph[dest_path[i]].name );
  }
  bwprintf( COM2, "\r\n" );

#endif 
}

