#include "rail_control.h"
#include "tools.h"
#include "track_data_new.h"
#include "track_node.h"
#include "global.h"

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
  cmds->sw_count = cmds->train_id = cmds->train_action = cmds->train_delay = \
  cmds->switch_id0= cmds->switch_action0= cmds->switch_delay0= \
  cmds->switch_id1= cmds->switch_action1= cmds->switch_delay1= \
  cmds->switch_id2= cmds->switch_action2= cmds->switch_delay2= 0;
}

void get_next_command( train_state_t* train, rail_cmds_t* cmds ) {
  track_node_t* track_graph = train->track_graph;
  int train_id = train->train_id;
  int src_id = train->prev_node_id;
  int dest_id = train->next_node_id;
  int speed_idx = ( train->cur_speed - train->prev_speed > 0 ) ? \
                    train->cur_speed : train->cur_speed + 15;
  int train_velocity = train->speeds[speed_idx].straight_vel; 
  //int stop_time = train->speeds[speed_idx].stopping_time;
  int stop_dist = train->speeds[speed_idx].stopping_distance;
  int safe_branch_dist = train->speeds[speed_idx].safe_branch_distance; 

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
    /* if the node is the end of the route */
    if( i == steps_to_dest - 1 ) {
      /* get dist between sensor and dest */  
      int sensor2dest_dist = all_dist[cur_node_id] - all_dist[prev_sensor_id]; 
      /* if we are the last sensor or the dest and second sensor is too close */
      if( prev_sensor_id == src_id || 
         ( prev_sensor_id == second_sensor_id && stop_dist > sensor2dest_dist )) {
        int src2dest_dist = all_dist[cur_node_id];
        
        cmds->train_id = train_id;
        cmds->train_action = TR_STOP;
        cmds->train_delay = (( src2dest_dist - stop_dist ) > 0 ) ? ( src2dest_dist - stop_dist ) / train_velocity : 0;
      }
    }
    /* update prev_sensor iff cur_sensor is the sensor immediately after src */
    else if( track_graph[cur_node_id].type == NODE_SENSOR && second_sensor_id == -1 ) {
      second_sensor_id = cur_node_id;
      prev_sensor_id = second_sensor_id;
    } 
    /* check reverse before branch, but handle reverse on branch separately */
    else if(( i - 1 ) >= 0 && ( track_graph[cur_node_id].reverse - track_graph == i - 1 )) {
      assert( 1, i - 1 >= 0 && i < steps_to_dest );

      cmds->train_id = train_id;
      cmds->train_action = TR_RV;
      cmds->train_delay = 0;

      /* deal with reverse on branch */
      int sensor2reverse_dist = all_dist[cur_node_id] - all_dist[prev_sensor_id];
      if( track_graph[cur_node_id].type == NODE_BRANCH && ( prev_sensor_id == src_id || 
            ( prev_sensor_id  == second_sensor_id && stop_dist > sensor2reverse_dist ))) {
        assert( 1, track_graph[i-1].type == NODE_MERGE );
        int src2reverse_dist = all_dist[cur_node_id];
        cmds->train_delay = (( src2reverse_dist - stop_dist ) > 0 ) ? ( src2reverse_dist - stop_dist ) / train_velocity : 0;
      }
    }
    /* finally, branching case */
    else if( track_graph[cur_node_id].type == NODE_BRANCH && i + 1 < steps_to_dest ) {
      debug( "branch_name: %s", track_graph[cur_node_id].name );
      assert( 1, switch_count < 3 );
      action = -1;

      #define set_switch( _cmds, _switch_count, _switch_id, _action, _delay ) \
        ++(_cmds->sw_count); \
        _cmds->switch_id##_switch_count = _switch_id; \
        _cmds->switch_action##_switch_count = _action; \
        _cmds->switch_delay##_switch_count = _delay; 
      /* get dist between sensor and branch*/  
      int sensor2branch_dist = all_dist[cur_node_id] - all_dist[prev_sensor_id]; 
      int sensor2branch_time = SW_TIME;//FIXME: replace it with actual time

      /* issue switch commands */
      if(( prev_sensor_id == src_id && sensor2branch_dist >= safe_branch_dist) ||
         ( prev_sensor_id == second_sensor_id && 
                              sensor2branch_dist < safe_branch_dist )) {
        assert( 1, switch_count < 3 );
        assertm( 1, track_graph[cur_node_id].num > 0 || 
                   track_graph[cur_node_id].num < 19 || 
                   track_graph[cur_node_id].num > 152 || 
                   track_graph[cur_node_id].num < 157, 
                   "num: %d", track_graph[cur_node_id].num );
        assert( 1, i + 1 < steps_to_dest );
        assert( 1, (( track_graph[cur_node_id].edge[DIR_STRAIGHT].dest == 
                    &track_graph[dest_path[i+1]] ) || 
                    ( track_graph[cur_node_id].edge[DIR_CURVED].dest == 
                      &track_graph[dest_path[i+1]] )));

        action = ( track_graph[cur_node_id].edge[DIR_STRAIGHT].dest == 
                   &track_graph[dest_path[i+1]] ) ? SW_STRAIGHT : SW_CURVED;

        if( switch_count == 0 ){
          set_switch( cmds, 0, track_graph[cur_node_id].num, action, sensor2branch_time - SW_TIME );
        }
        else if( switch_count == 1 ) {
          set_switch( cmds, 1, track_graph[cur_node_id].num, action, sensor2branch_time - SW_TIME);
        }
        else if( switch_count == 2 ) {
          set_switch( cmds, 2, track_graph[cur_node_id].num, action, sensor2branch_time - SW_TIME );
        }
        else
          assert( 1, false );
        
        ++switch_count;
      }
      else 
        assert( 1, false );
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

  assert( 1, last->dist < INT_MAX );
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
  bwprintf( COM2, "size: %d\n\r", min_heap->size );
  int idx;
  for( idx = 0; idx < min_heap->size; ++idx ) {
    bwprintf( COM2, "id: %d, dist: %d", min_heap->nodes[idx].id, min_heap->nodes[idx].dist );
  }
  bwprintf( COM2, "\n\r" );
}

void dijkstra( struct track_node* track_graph, int src_id, int* path, int* dist, int* step ) {
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
      test_dist = dist[track_id];
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
  bwprintf( COM2, "%d steps from %s to %s:\t%s", steps, track_graph[src_id].name,
      track_graph[dest_id].name, track_graph[src_id].name );
  for( i = 0; i < steps; ++i ) {
    bwprintf( COM2, "->%s", track_graph[dest_path[i]].name );
  }
  bwprintf( COM2, "\r\n" );
}

