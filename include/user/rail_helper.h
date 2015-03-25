#ifndef __RAIL_HELPER_H__
#define __RAIL_HELPER_H__

#include "global.h"

#define SWITCH_BUFFER 150 // 10cm buffer
#define STOP_BUFFER 100 // 10cm buffer
#define STOP_TIME_BUFFER 2

struct _train_state_;
struct _track_node_;

int get_rand_dest( int super_complicated_seed, struct _track_node_ *graph, int cur_sensor_id );

void clear_reservations( struct _track_node_ *graph );

void clear_reservations_by_train( struct _track_node_ *graph, struct _train_state_ *train );

void clear_prev_train_reservation( struct _train_state_ *train );

void print_rsv( struct _train_state_ *train, struct _train_state_ *trains );

int get_train_idx( int train_num );

// time to switch branch * velocity of train + buffer + length of train
int safe_distance_to_branch( struct _train_state_ *train );

int safe_distance_to_sensor( );

// does NOT take into reverse/speed changes
int time_to_node( struct _train_state_ *train, int dist_to_node, int cur_time );

int get_mm_past_last_landmark( struct _train_state_ *train, int cur_time );

int get_next_sensor( struct _train_state_ *train, int *dist_to_next_sensor );

void update_velocity( struct _train_state_ *train, int cur_time, int prev_time, int dist );

void init_switches( int *switch_states );

void init_trains( struct _train_state_ *trains, struct _track_node_* track_graph, int* switch_states );

int get_cur_velocity( struct _train_state_ *train, int cur_time );

int get_delay_time_to_stop( struct _train_state_ *train, int dist );

int get_cur_stopping_distance( struct _train_state_ *train );

int get_cur_stopping_time( struct _train_state_ *train );

inline int get_expected_train_idx( struct _train_state_* trains, int sensor_num );

inline int safe_distance_to_stop( struct _train_state_ *train );

inline int get_len_train_ahead( struct _train_state_ *train );

inline int get_len_train_behind( struct _train_state_ *train );

void init_58( struct _train_state_ *train );

void init_45( struct _train_state_ *train );

void init_24( struct _train_state_ *train );

void init_12( struct _train_state_ *train );

#endif
