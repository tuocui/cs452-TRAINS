#ifndef __RAIL_HELPER_H__
#define __RAIL_HELPER_H__

#include "global.h"

#define SWITCH_TIME 250
#define SWITCH_BUFFER 20 // 2cm buffer
#define STOP_BUFFER 20

struct _train_state_;

// time to switch branch * velocity of train + buffer + length of train
int safe_distance_to_branch( struct _train_state_ *train );

int safe_distance_to_sensor( );

// does NOT take into reverse/speed changes
int time_to_next_node( struct _train_state_ *train, int next_node );

int time_to_next_sensor( struct _train_state_ *train, int *switches );

int get_mm_past_last_landmark( struct _train_state_ *train, int cur_time );

#endif
