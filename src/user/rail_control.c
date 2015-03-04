#include "rail_control.h"
#include "tools.h"
#include "track_data_new.h"
#include "track_node.h"
#include "global.h"

//TODO: just realize the "act upon every sensor hit" approach does not work
//because some sensors are too close to the branches, so there is absolutely not
//enough time to issue a turn-out switch. It does not matter for milestone 1, but
//this approach cannot achive monitoring two trains.
///* get_next_command calls graph search to get the shortest path,
// * for now, if the path only has length of one, the we issue stop command.
// * if there are more than one node, it finds out if there needs reverse/switch
// * If two nodes are reverse of each other,
// * we issue reverse command. If the first node is MR, we check the second
// * to decide which direction to go to. 
// * The function returns a list of commands back to the server. For now, the 
// * return value is an array of pair of integers. The even-positioned integers
// * represent the operations (such as reverse, set speed to 13 etc), the odd-
// * positioned integers represent the delay time before the operation. For 
// * example, if the result is [RV, 120, SW13C, 150, EOC], it means we want to 
// * delay 120 msec before the reverse command, and delay 150 msec before we 
// * switch turn-out 13 to curved. Because the calling function does not know 
// * exactly how many commands there are (can be 0-2) we use macro EOC to 
// * indicate the end of commands.
// *
// * FIXME: we actually should check reverse condition many nodes ahead, but
// * only execute it when we reach the sensor immediately before the reverse node
// */
//
//void get_next_command( track_node_t* track_graph, int src_id, int dst_id, int* cmds ) {
//  assert( 1, track_graph && src_id >= 0 && src_id < TRACK_MAX && dst_id >= 0 && dst_id < TRACK_MAX );
//
//  /* run dijkstra on the src and dst */
//  int all_path[NODE_MAX], all_dist[NODE_MAX], all_step[NODE_MAX];
//  dijkstra( track_graph, src_id, all_path, all_dist, all_step );
//  
//  /* get shortest path for our destination */
//  int dst_path[all_step[dst_id]];
//  get_shortest_path( all_path, all_step, src_id, dst_id, dst_path );
//  //TODO: remove the debug print
//  int i;
//  for( i = 0; i < all_step[dst_id]; ++i ) {
//    bwprintf( COM2, "->%s", track_graph[dst_path[i]].name );
//  }
//  //TODO: end of debug;
//  
//  /* for now, if there is only one node ahead, issue stop command, this behavior
//   * should be changed after milestone 1 
//   */
//  int cmd_pos;
//  if( all_step[dst_id] <= 1 ) {
//    cmds[cmd_pos++] = TR_STOP;
//    cmds[cmd_pos++] = 0;
//    cmds[cmd_pos++] = EOC;
//  }
//  else {
//    int node_id1 = dst_path[0]; 
//    int node_id2 = dst_path[1];
//    /* check reverse condition, this approach is wrong, we need to factor in time */
//    if( track_graph[node_id1].reverse - track_graph == node_id2 ) {
//      assertm( 1, track_graph[node_id2].reverse - track_graph == node_id1,
//          "node_id1: %d, node_id2: %d", node_id1, node_id2 );
//      //FIXME: below time should be calculated
//      int reverse_delay = 3000;
//      int reverse_time = 5000; 
//      int switch_delay = reverse_delay + reverse_time;
//      cmds[cmd_pos++] = TR_RV;
//      cmds[cmd_pos++] = reverse_delay;
//      cmds[cmd_pos++] = switch_delay;
//    }
//    /* check merge condition */
//  
//}


//TODO: put below into a separate file "track_search"
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
    init_node( &(min_heap->nodes[i]), i, INT_MAX);
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
  bwprintf( COM2, "size: %d", min_heap->size );
  int idx;
  for( idx = 0; idx < min_heap->size; ++idx ) {
    bwprintf( COM2, "id: %d, dist: %d", min_heap->nodes[idx].id, min_heap->nodes[idx].dist );
  }
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

void get_shortest_path( int* all_path, int* all_step, int src_id, int dst_id, int* dst_path ) {
  int steps = all_step[dst_id];
  while( steps ) {
    dst_path[steps] = dst_id;

    if( dst_id == src_id )
      break;

    dst_id = all_path[dst_id];
    --(steps);
  }
}

void print_shortest_path( track_node_t* track_graph, int* all_path, int* all_step, int src_id, int dst_id, int* dst_path ) {
  debug( "src_id: %d, dst_id: %d", src_id, dst_id );
  int tmp_id = dst_id;
  int steps = all_step[tmp_id] - 1;
  while( steps >= 0 ) {
    dst_path[steps] = tmp_id;

    if( tmp_id == src_id )
      break;

    tmp_id = all_path[tmp_id];
    --(steps);
  }

  steps = all_step[dst_id];
  int i;
  bwprintf( COM2, "%d steps from %s to %s:\t%s", steps, track_graph[src_id].name,
      track_graph[dst_id].name, track_graph[src_id].name );
  for( i = 0; i < steps; ++i ) {
    bwprintf( COM2, "->%s", track_graph[dst_path[i]].name );
  }
  bwprintf( COM2, "\n\r" );
}
