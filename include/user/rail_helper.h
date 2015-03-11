#ifndef __RAIL_HELPER_H__
#define __RAIL_HELPER_H__

#include "global.h"

#define SWITCH_BUFFER 150 // 10cm buffer
#define STOP_BUFFER 150 // 10cm buffer
#define STOP_TIME_BUFFER 2

struct _train_state_;
struct _track_node_;

// time to switch branch * velocity of train + buffer + length of train
int safe_distance_to_branch( struct _train_state_ *train );

int safe_distance_to_sensor( );

// does NOT take into reverse/speed changes
int time_to_next_node( struct _train_state_ *train, int next_node );

int time_to_next_sensor( struct _train_state_ *train, int *switches );

int get_mm_past_last_landmark( struct _train_state_ *train, int cur_time );

int get_next_sensor( struct _train_state_ *train, int *dist_to_next_sensor );

void update_velocity( struct _train_state_ *train, int cur_time, int prev_time, int dist );

void init_switches( int *switch_states );

void init_trains( struct _train_state_ *trains, struct _track_node_* track_graph, int* switch_states );

int get_cur_velocity( struct _train_state_ *train, int cur_time );

int get_cur_stopping_distance( struct _train_state_ *train );

int get_cur_stopping_time( struct _train_state_ *train );

inline int get_expected_train_idx( struct _train_state_* trains, int sensor_num );

#endif
