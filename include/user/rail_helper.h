#ifndef __RAIL_HELPER_H__
#define __RAIL_HELPER_H__

#include "global.h"
#include "calibration.h"

#define SWITCH_TIME 250
#define SWITCH_BUFFER 20 // 2cm buffer
#define STOP_BUFFER 20

// time to switch branch * velocity of train + buffer + length of train
int safe_distance_to_branch( struct train_t *train );

int safe_distance_to_sensor( );

// does NOT take into reverse/speed changes
int time_to_next_node( struct train_t *train, int next_node );

int time_to_next_sensor( struct train_t *train, int *switches );

#endif
