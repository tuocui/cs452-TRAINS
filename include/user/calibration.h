#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "global.h"

#define NUM_SPEEDS 30

typedef struct speed_info_t {
  int speed;
  int high_low;
  int str_vel;
  int curved_vel;
  int stopping_distance;
  int stopping_time;
  int accel_distance;
  int accel_time;
} speed_info_t;

/* feel free to add to this */
typedef struct train_t {
  int prev_landmark;
  int next_landmark;
  int nm_past_landmark;
  int time_at_last_landmark;
  int dest_landmark;
  int cur_speed;
  int prev_speed;
  int speed_change_time;
  int fallback_sensors[2];
  int pickup_len;
  int length;
  struct speed_info_t speeds[NUM_SPEEDS]; // Two different velocities per speed.
} train_t;

void calibrate_train_velocity( );
void calibrate_stopping_distance( );
void calibrate_accel_time( );
void init_trains( train_t *trains, int num_trains );

#endif
