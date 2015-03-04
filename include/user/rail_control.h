#ifndef __RAIL_CONTROL_H__
#define __RAIL_CONTROL_H__

#include "global.h"

#define EOC       -1 
#define TR_STOP   0
#define TR_SPD_1  1 
#define TR_SPD_2  2 
#define TR_SPD_3  3 
#define TR_SPD_4  4 
#define TR_SPD_5  5 
#define TR_SPD_6  6 
#define TR_SPD_7  7 
#define TR_SPD_8  8 
#define TR_SPD_9  9 
#define TR_SPD_10 10 
#define TR_SPD_11 11 
#define TR_SPD_12 12 
#define TR_SPD_13 13 
#define TR_SPD_14 14 
#define TR_RV     15

#define SW1_C     100 
#define SW2_C     200 
#define SW3_C     300
#define SW4_C     400
#define SW5_C     500
#define SW6_C     600
#define SW7_C     700
#define SW8_C     700
#define SW9_C     900
#define SW10_C   1000
#define SW11_C   1100
#define SW12_C   1200
#define SW13_C   1300
#define SW14_C   1400
#define SW15_C   1500
#define SW16_C   1600
#define SW17_C   1700
#define SW18_C   1800
#define SW153_C  1530
#define SW154_C  1540
#define SW155_C  1550
#define SW156_C  1560

#define SW1_S     101 
#define SW2_S     201 
#define SW3_S     301
#define SW4_S     401
#define SW5_S     501
#define SW6_S     601
#define SW7_S     701
#define SW8_S     701
#define SW9_S     901
#define SW10_S   1001
#define SW11_S   1101
#define SW12_S   1201
#define SW13_S   1301
#define SW14_S   1401
#define SW15_S   1501
#define SW16_S   1601
#define SW17_S   1701
#define SW18_S   1801
#define SW153_S  1531
#define SW154_S  1541
#define SW155_S  1551
#define SW156_S  1561

struct track_node;

/* note: this is not a complete min heap library, it does not support inserting
 * any number to the heap. This heap is modified to better suit the needs of dijkstra's
 * algorithm. On initialization, the heap's all elements are assigned to INT_MAX, since
 * the distance of dijkstra's algorithm only gets decreased, we bubble up the node 
 * whose distance is decreased. Thus, this heap does not need to support item insertion.
 */

typedef struct _min_heap_node_ {
  int id;       // must be the same as the id for each track vertex
  int dist;     // distance to the source vertex
} min_heap_node_t;

typedef struct _min_heap_ {
  int size;     // num of nodes in the heap
  int capacity; // max num of nodes 
  int * node_id2idx; // map node_id(track_vertex) to heap idx 
  min_heap_node_t * nodes; // heap representation by an array
} min_heap_t;

inline void init_node( min_heap_node_t * node, int id, int dist );

void init_min_heap( min_heap_t * min_heap, int src_id, int * node_id2idx, min_heap_node_t * nodes );

inline void swap_node( min_heap_node_t * node_a, min_heap_node_t * node_b );

void make_min_heap( min_heap_t * min_heap, int idx );

inline bool heap_empty( min_heap_t * min_heap );

inline bool heap_find( min_heap_t * min_heap, int id );

min_heap_node_t * extract_min( min_heap_t * min_heap );

inline void print_min_heap( min_heap_t * min_heap );

void dijkstra( struct track_node* track_graph, int src_id, int* all_path, int* all_dist, int* all_step );

void get_shortest_path( int* all_path, int* all_step, int src_id, int dst_id, int* dst_path );

void print_shortest_path( struct track_node * track_graph, int* all_path, int* all_step, int src_id, int dst_id, int* dst_path );

void decrease_dist( min_heap_t * min_heap, int id, int dist );

#endif
