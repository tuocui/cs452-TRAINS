#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "global.h"



/* feel free to add to this */
struct _train_state_;

void calibrate_train_velocity( );
void calibrate_stopping_distance( );
void calibrate_accel_time( );
void init_trains( struct _train_state_ *trains, int num_trains );

#endif
