#ifndef __RAIL_CONTROL_H__
#define __RAIL_CONTROL_H__

#include "global.h"

#define NUM_SPEEDS  30
#define DIST_MAX    200000

#define TR_MAX 4
#define TR_PREF_SPEED 10

#define TRAIN_MAX   4
#define TRAIN_58_IDX 0
#define TRAIN_24_IDX 1
#define TRAIN_63_IDX 2
#define TRAIN_45_IDX 3

#define TRAIN_58_NUM 58
#define TRAIN_24_NUM 24
#define TRAIN_63_NUM 63
#define TRAIN_45_NUM 45

#define convert_train_num2idx( _train_num ) \
  ( _train_num == TRAIN_58_NUM ? TRAIN_58_IDX : \
    _train_num == TRAIN_24_NUM ? TRAIN_24_IDX : \
    _train_num == TRAIN_63_NUM ? TRAIN_63_IDX : \
    _train_num == TRAIN_45_NUM ? TRAIN_45_IDX : NONE )

#define NONE      -1
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

#define SW_STRAIGHT 0 
#define SW_CURVED   1 

#define SW_MAX  23
#define SW1     1 
#define SW2     2 
#define SW3     3
#define SW4     4
#define SW5     5
#define SW6     6
#define SW7     7
#define SW8     8
#define SW9     9
#define SW10   10
#define SW11   11
#define SW12   12
#define SW13   13
#define SW14   14
#define SW15   15
#define SW16   16
#define SW17   17
#define SW18   18
#define SW153  19 
#define SW154  20 
#define SW155  21 
#define SW156  22 

#define get_switch_twin( _switch_id ) \
  ( _switch_id == SW153 ? SW154 : \
    _switch_id == SW154 ? SW153 : \
    _switch_id == SW155 ? SW156 : \
    _switch_id == SW156 ? SW155 : NONE )

#define SW_TIME     100 // in ms

#define TR_STOP           0
#define TR_REVERSE        1
#define TR_CHANGE_SPEED   2
#define TR_INIT           3
#define TR_DEST           4
#define TR_ACCEL          5
#define TR_DECEL          6
#define TR_CH_DIR         7
#define TRACK_RSV         8
#define TR_RAND_DEST      9
#define TR_USER_CTRL      10
#define TR_SPEED_UP       11
#define TR_SLOW_DOWN      12
#define TR_SWITCH_LEFT    13
#define TR_SWITCH_RIGHT   14
#define TR_STOP_USER_CTRL 15
#define TR_SET_RAND_DEST  16
#define TR_PRIORITY       17

#define NUM_FALLBACK 5
#define SENSOR_STACK_MAX 10 
#define SW_CMD_MAX   4

#define USER_INPUT_NUM  99
#define DEFAULT_TRAIN_LEN 210

#define CMD_QUEUE_MAX 5

#define CONVERT_SWITCH_ID( _switch_num ) \
  if( _switch_num > 152 ) { \
    _switch_num -= 134; \
  } else { ; }

struct _track_node_;

typedef struct _generic_cmd_{
  int id;
  int action;
  int delay;
  
  int train_speed;
  int train_dest;
  int train_mm_past_dest;
  int train_accel;
  int train_decel;
} generic_cmd_t;

typedef struct _generic_cmds_list_ {
  int count;
  struct _generic_cmd_ cmds[CMD_QUEUE_MAX];
} generic_cmds_list_t;

typedef struct _switch_cmd_ {
  int switch_id;
  int switch_action;
  int switch_delay;
} switch_cmd_t;

typedef struct _rail_cmds_ {
  int train_id;
  int train_action;
  int train_delay;

  int train_speed;
  int train_dest;
  int train_mm_past_dest;
  int train_accel;
  int train_decel;

  int rsv_node_id;
  int rsv_node_dir;
  
  int rail_cmd_switch_idx;
  switch_cmd_t switch_cmds[SW_CMD_MAX];
} rail_cmds_t;

typedef struct _speed_info_ {
  int speed; //TODO
  int high_low;
  int straight_vel;
  int curved_vel;
  int stopping_distance;
  int stopping_time;
  int safe_branch_distance;
  int accel_distance;
  int accel_time;
} speed_info_t;

typedef struct _train_state_ {
  struct _track_node_* track_graph;
  int * switch_states;

  enum {
    NOT_INITIALIZED = -1,
    READY = 0,
    BUSY,
    STOPPING,
    ACCELERATING,
    REVERSING,
    INITIALIZING,
    HANDLING_COLLISION,
  } state;

  enum {
    NO_PRIORITY = -1,
    SHUTTLE = 0,
    LIMO = 1,
    TANK = 2,
  } priority;

  /* used as a queue */
  int dest_path[NODE_MAX];
  int dest_path_cur_idx;
  int all_dist[NODE_MAX];
  int dest_total_steps;

  /* used as a stack */
  int rv_sensor_stack[SENSOR_STACK_MAX];
  int rv_sensor_stack_idx;

  bool fallback_sensor_hit;
  bool train_reach_destination;
  int train_id;
  int prev_sensor_id;
  int next_sensor_id;
  //int time_to_next_sensor_abs; // time_to_next_sensor + cur_time
  int time_to_next_sensor;
  int dest_id;
  int prev_dest_id;
  int front_len;
  int back_len;
  int mm_past_dest;
  int dist_to_next_sensor;
  int time_at_last_landmark;
  int time_since_last_pos_update;
  int vel_at_last_pos_update;
  int vel_at_last_landmark;
  int mm_past_landmark;
  int length;
  int pickup_len;
  int decel_rate;       // mm/s^2
  int accel_rate;
  int cur_speed;
  int prev_speed;
  int cur_vel;
  int speed_change_time;
  int init_time;
  bool is_forward;
  int rev_branch_ignore;
  int next_branch_num;
  int fallback_sensors[NUM_FALLBACK];
  int fallback_dist[NUM_FALLBACK];
  int time_to_fallback_sensor[NUM_FALLBACK];
  bool set_rand_dest;
  bool user_controlled;
  bool warning_displayed;
  speed_info_t speeds[NUM_SPEEDS]; // Two different velocities per speed.
} train_state_t;

void init_rail_cmds( rail_cmds_t* cmds );

void request_next_command( train_state_t* train, rail_cmds_t* cmds );

inline void compute_next_command( train_state_t *train, rail_cmds_t* cmds );
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

int update_track_reservation( train_state_t *train, train_state_t *all_trains );

inline void init_node( min_heap_node_t * node, int id, int dist );

void init_min_heap( min_heap_t * min_heap, int src_id, int * node_id2idx, min_heap_node_t * nodes );

inline void swap_node( min_heap_node_t * node_a, min_heap_node_t * node_b );

void make_min_heap( min_heap_t * min_heap, int idx );

inline bool heap_empty( min_heap_t * min_heap );

inline bool heap_find( min_heap_t * min_heap, int id );

void update_prev_sensor_id_for_rev( train_state_t *train );

min_heap_node_t * extract_min( min_heap_t * min_heap );

inline void print_min_heap( min_heap_t * min_heap );

void dijkstra( struct _track_node_* track_graph, int train_id, int src_id, int* all_path, int* all_dist, int* all_step );

void extract_shortest_path( int* all_path, int* all_step, int src_id, int dst_id, int* dst_path );

inline void get_shortest_path( train_state_t *train );

void print_shortest_path( struct _track_node_ * track_graph, int* all_path, int* all_step, int src_id, int dst_id, int* dst_path );

void print_shortest_dist( struct _train_state_ * train);

void print_train_path( struct _train_state_ * train );

void print_cmds( struct _rail_cmds_ * cmds );

void decrease_dist( min_heap_t * min_heap, int id, int dist );

void predict_next_sensor_static( train_state_t *train_state );

void predict_next_fallback_sensors_static( train_state_t *train );

void predict_next_sensor_dynamic( train_state_t *train_state );

inline void pack_train_cmd( rail_cmds_t *cmds, int train_id, int ACTION, int delay );

inline void pack_switch_cmd( rail_cmds_t *cmds, int switch_id, int ACTION, int delay );


#endif
