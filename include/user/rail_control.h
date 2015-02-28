#ifndef __RAIL_CONTROL_H__
#define __RAIL_CONTROL_H__

#define NODE_MAX 140

typedef struct _min_heap_node_ {
  int id;
  int dist;
} min_heap_node_t;

typedef struct _min_heap_ {
  int size;     // num of nodes contains
  int capacity; // max num of nodes
  int node_id2pos[NODE_MAX]; // map node id to its position in the heap array 
  min_heap_node_t * nodes[NODE_MAX]; // heap representation by an array
} min_heap_t;


#endif
