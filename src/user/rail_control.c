#include "rail_control.h"
#include "tools.h"
#include "track_data_new.h"
#include "track_node.h"

inline void init_node( min_heap_node_t * node, int id, int dist ) {
  assert( 1, node || id >= 0 || id < NODE_MAX );

  node->id = id;
  node->dist = dist;
}

void init_min_heap( min_heap_t * min_heap, int * node_id2idx, min_heap_node_t * nodes ) {
  assert( 1, min_heap );

  min_heap->size = 0;
  min_heap->capacity = NODE_MAX;
  min_heap->node_id2idx = node_id2idx;
  min_heap->nodes = nodes;

  int i;
  for( i = 0; i < min_heap->capacity; ++i ) {
    min_heap->node_id2idx[i] = i;
  }

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

  return root;  
}

void decrease_dist( min_heap_t * min_heap, int id, int dist ) {
  assert( 1, min_heap );
  assert( 1, id >= 0 && id <= min_heap->size );
  
  /* find the node with the vertex id */
  int idx = min_heap->node_id2idx[id];
  assert( 1, min_heap->nodes[idx].dist > dist );
  min_heap->nodes[idx].dist = dist;

  /* bubble up the node with the shorter distance */
  while( idx != 0 && min_heap->nodes[idx].dist < min_heap->nodes[(idx-1)/2].dist ) {
    int parent_idx = ( idx - 1 ) / 2;
    /* swap the idx */
    min_heap->node_id2idx[parent_idx] = idx;
    min_heap->node_id2idx[idx] = parent_idx;

    /* swap this node with its parent */
    swap_node( &( min_heap->nodes[idx] ), &( min_heap->nodes[parent_idx] ));
    
    idx = parent_idx;
  }
}

inline void print_min_heap( min_heap_t * min_heap ) {
  assert( 1, min_heap );
  debug( "size: %d", min_heap->size );
  int idx;
  for( idx = 0; idx < min_heap->size; ++idx ) {
    debug( "idx: %d, dist: %d", min_heap->nodes[idx].id, min_heap->nodes[idx].dist );
  }
}

int dijkstra( struct track_node * track_graph, int source ) {
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

