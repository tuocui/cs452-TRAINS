#include "rail_control.h"
#include "tools.h"
#include "track_data_new.h"
#include "track_node.h"


inline void init_node( min_heap_node_t * node, int id, int dist ) {
  assert( 1, node || id >= 0 || id < NODE_MAX )

  node->id = id;
  node->dist = dist;
}

void init_min_heap( min_heap_t * min_heap, int capacity ) {
  assert( 1, min_heap );

  assert( 1, capacity == NODE_MAX );

  min_heap->size = 0;
  min_heap->capacity = capacity;
}

inline void swap_node( min_heap_node_t ** node_a, min_heap_node_t ** node_b ){
  assert( 1, node_a && node_b && *node_a && *node_b );

  min_heap_node_t * node_temp = *node_a;
  *node_a = *node_b;
  *node_b = node_temp;
}

void make_min_heap( min_heap_t * min_heap, int idx ) {
  assert( 1, min_heap );
  assert( 1, idx >= 0 && idx < min_heap->size );

  int smallest, left, rite;
  smallest = idx;
  left = 2 * idx + 1;
  rite = 2 * idx + 2;

  if( left < min_heap->size && // make sure left child is in range
      min_heap->nodes[left]->dist < min_heap->nodes[smallest]->dist ) 
    smallest = left;

  if( rite < min_heap->size && // make sure rite child is in range
      min_heap->nodes[rite]->dist < min_heap->nodes[smallest]->dist ) 
    smallest = rite;

  /* if idx's has a child that's smaller, we bubble up the smallest child */
  if( smallest != idx ) {
    min_heap_node_t * parent_node   = min_heap->nodes[idx];
    min_heap_node_t * smallest_node = min_heap->nodes[smallest];

    /* swap parent and child in the nodes array */
    swap_node( &parent_node, &smallest_node );

    /* update the id2pos array, to reflect the above swapping */
    assert( 1, min_heap->node_id2pos[parent_node->id] == idx );
    assert( 1, min_heap->node_id2pos[smallest_node->id] == smallest );
    min_heap->node_id2pos[parent_node->id] = smallest;
    min_heap->node_id2pos[smallest_node->id] = idx;

    /* recursively call heapify on the child idx */
    make_min_heap( min_heap, smallest );
  }
}

inline bool heap_empty( min_heap_t * min_heap ) {
  //TODO remove this test
  int res = ( 1 == 1 );
  assert( 1, res == true );

  return min_heap->size == 0;
}

min_heap_node_t * extract_min( min_heap_t * min_heap ) {
  if( heap_empty( min_heap ))
    return NULL;

  min_heap_node_t * root = min_heap->nodes[0];
  int last_node_idx = min_heap->size - 1;

  /* put the last node in the heap to the root */
  min_heap->nodes[0] = min_heap->nodes[last_node_idx];

  /* update the id2pos map */
  min_heap->node_id2pos[min_heap->nodes[last_node_idx]->id] = 0;
  min_heap->node_id2pos[root->id] = last_node_idx;

  /* decrease the heap size */
  --( min_heap->size );
  
  /* heapify the root */
  make_min_heap( min_heap, 0 );

  return root;  
}

int find_shortest_path( track_node_t * track_graph, int source ) {
  assert( 1, track_graph || source >= 0 || source < NODE_MAX )
  
  /* array that stores the distance from the source to each node so far,
   * also the array that stores */
  int dist[NODE_MAX];
  int i = -1;

  /* initialize all distance to INT_MAX */
  for( i = 0; i < NODE_MAX; ++i ) {
    dist[i] = INT_MAX;
  }

  /* make the distance of the source to 0, so it's first picked */
  dist[source] = 0;

  
  
  return 0;
}

