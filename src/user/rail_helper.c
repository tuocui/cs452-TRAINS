#include "io.h"
#include "tools.h"
#include "track.h"
#include "clock_server.h"
#include "syscall.h"
#include "rail_helper.h"
#include "calibration.h"
#include "track_node.h"
#include "rail_control.h"

// time to switch branch * velocity of train + buffer + length of train
// Assume 250ms to switch switch
// returns distance in mm
int safe_distance_to_branch( train_state_t *train ) {
  int vel = (train->speeds[train->cur_speed]).straight_vel;
  return ( vel * SW_TIME ) / 100000 + SWITCH_BUFFER + train->length;
}

// stopping distance + buffer + length of train
// return distance in mm
int safe_distance_to_stop( train_state_t *train ) {
  return (train->speeds[train->cur_speed]).stopping_distance + STOP_BUFFER + train->length;
}

// returns in ms
int get_accel_time( int cur_speed, int prev_speed ) {
  int speed_diff = cur_speed - prev_speed;
  int offset = 300;
  if( speed_diff < 0 ) {
    speed_diff = 0 - speed_diff;
    offset = 500;
  }
  if( speed_diff == 1 ) {
    return offset;
  }
  return ( 300 * speed_diff ) - offset;
}

// returns in ms
// dist in mm
// vel in mm/100s
int time_to_dist_constant_vel( int dist, int vel ) {
  return (dist * 100000) / vel;
}

int get_mm_past_last_landmark( train_state_t *train, int cur_time ) {
  int speed_change_time = train->speed_change_time * 10;
  int prev_sensor_time = train->time_at_last_landmark * 10;
  int speed_finish_time = speed_change_time + get_accel_time( train->cur_speed, train->prev_speed );
  int cur_vel = train->speeds[train->cur_speed].straight_vel;
  int prev_vel = train->speeds[train->prev_speed].straight_vel;
  cur_time *= 10;
  if( speed_finish_time < prev_sensor_time ) {
    return ( ( cur_time - prev_sensor_time ) * cur_vel ) / 100000;
  } else if( speed_finish_time < cur_time ) {
    return ( ( cur_time - speed_finish_time ) * cur_vel ) / 100000 + 
           ( ( speed_finish_time - prev_sensor_time ) * prev_vel ) / 100000 + 
           ( ( speed_finish_time - prev_sensor_time ) * ( cur_vel - prev_vel ) ) / 200000;
  } else {
    return ( ( cur_time - prev_sensor_time ) * prev_vel ) / 100000 + 
           ( ( cur_time - prev_sensor_time ) * ( cur_vel - prev_vel ) ) / 200000;
  }
}

int get_cur_vel( train_state_t *train, int cur_time ) {
  return 123;
}

// node is index
// returns time in ms
// dist = distance from previous landmark it passed to the node
int time_to_node( train_state_t *train, int dist_to_node, track_node_t *track ) {
  int cur_time = Time( ) * 10;
  int speed_change_time = train->speed_change_time * 10;

  int accel_time = get_accel_time( train->cur_speed, train->prev_speed );
  int speed_finish_time = speed_change_time + accel_time;
  // Finished accel/decel before cur_time?
  int mm_past = get_mm_past_last_landmark( train, cur_time );
  dist_to_node -= mm_past;
  if( speed_finish_time < cur_time ) {
    return time_to_dist_constant_vel( dist_to_node, train->speeds[train->cur_speed].straight_vel );
  } else { // still changing velocity? fuck TODO: Figure this shit out
    int cur_velocity = ( (train->speeds[train->cur_speed].straight_vel + 
                          train->speeds[train->prev_speed].straight_vel) * 100 ) / 
                          ( ( speed_finish_time * 100 ) / cur_time );
    int dist_to_const_vel = ( ( speed_finish_time - cur_time ) * cur_velocity ) / 10000 + 
                            ( ( speed_finish_time - cur_time ) * ( train->speeds[train->cur_speed].straight_vel - cur_velocity ) ) / 20000;
    // Will finish acceleration after we hit the node
    if( dist_to_const_vel > dist_to_node ) {
      return ( 200000 * dist_to_node ) / (cur_velocity + train->speeds[train->cur_speed].straight_vel );
    } else { // will finish acceleration before we hit the node
      return speed_finish_time + time_to_dist_constant_vel( dist_to_node - dist_to_const_vel, train->speeds[train->cur_speed].straight_vel );
    }
  }
}

void init_trains( train_state_t *trains, int num_trains ) {
  int i;
  int j;
  for( i = 0; i < num_trains; ++i ) {
    trains[i].prev_sensor_id= 0;
    trains[i].next_sensor_id= 0;
    trains[i].mm_past_landmark = 0;
    trains[i].cur_speed = 0;
    for( j = 0; j < NUM_SPEEDS; ++j ) {
      trains[i].speeds[j].speed = 0;
      trains[i].speeds[j].high_low = 0;
      trains[i].speeds[j].straight_vel = 0;
      trains[i].speeds[j].curved_vel = 0;
      trains[i].speeds[j].stopping_distance = 0;
      trains[i].speeds[j].stopping_time = 0;
      trains[i].speeds[j].accel_distance = 0;
      trains[i].speeds[j].accel_time = 0;
    }
  }

  /* Copypasta calibration output here */
  // Velocity in mm/100s, divide by 1000 to get cm/s
  // Stopping distance in mm
  // (2*d)/v0 d = stopping distance, v0 = velocity
  trains[TRAIN_58].length = 210;
  trains[TRAIN_58].pickup_len = 50;
  trains[TRAIN_58].train_id = 58;
  trains[TRAIN_58].prev_sensor_id = 0;
  trains[TRAIN_58].next_sensor_id = 0;
  trains[TRAIN_58].dest_id = 0;
  trains[TRAIN_58].dist_to_next_sensor = 0;
  trains[TRAIN_58].time_at_last_landmark = 0;
  trains[TRAIN_58].mm_past_landmark = 0;
  trains[TRAIN_58].cur_speed = 0;
  trains[TRAIN_58].prev_speed = 0;
  trains[TRAIN_58].speed_change_time = 0;
  trains[TRAIN_58].is_forward = 1;
  trains[TRAIN_58].state = NOT_INITIALIZED;
  trains[TRAIN_58].speeds[14].straight_vel = 50579; // 14 HIGH
  trains[TRAIN_58].speeds[14].curved_vel = 50586;
  trains[TRAIN_58].speeds[14].stopping_distance = 1188;
  trains[TRAIN_58].speeds[13].straight_vel = 50592; // 13 HIGH
  trains[TRAIN_58].speeds[13].curved_vel = 50583;
  trains[TRAIN_58].speeds[13].stopping_distance = 1052;
  trains[TRAIN_58].speeds[12].straight_vel = 48798; // 12 HIGH
  trains[TRAIN_58].speeds[12].curved_vel = 48517;
  trains[TRAIN_58].speeds[12].stopping_distance = 852;
  trains[TRAIN_58].speeds[11].straight_vel = 41440; // 11 HIGH
  trains[TRAIN_58].speeds[11].curved_vel = 41749;
  trains[TRAIN_58].speeds[11].stopping_distance = 645;
  trains[TRAIN_58].speeds[10].straight_vel = 33814; // 10 HIGH
  trains[TRAIN_58].speeds[10].curved_vel = 34627;
  trains[TRAIN_58].speeds[10].stopping_distance = 460;
  trains[TRAIN_58].speeds[9].straight_vel = 28004; // 9 HIGH
  trains[TRAIN_58].speeds[9].curved_vel = 27860;
  trains[TRAIN_58].speeds[9].stopping_distance = 336;
  trains[TRAIN_58].speeds[8].straight_vel = 22133; // 8 HIGH
  trains[TRAIN_58].speeds[8].curved_vel = 22370;
  trains[TRAIN_58].speeds[8].stopping_distance = 231;
  trains[TRAIN_58].speeds[23].straight_vel = 18907; // 8 LOW
  trains[TRAIN_58].speeds[23].curved_vel = 19127;
  trains[TRAIN_58].speeds[23].stopping_distance = 186;
  trains[TRAIN_58].speeds[24].straight_vel = 24265; // 9 LOW
  trains[TRAIN_58].speeds[24].curved_vel = 24270;
  trains[TRAIN_58].speeds[24].stopping_distance = 289;
  trains[TRAIN_58].speeds[25].straight_vel = 31219; // 10 LOW
  trains[TRAIN_58].speeds[25].curved_vel = 31030;
  trains[TRAIN_58].speeds[25].stopping_distance = 402;
  trains[TRAIN_58].speeds[26].straight_vel = 37521; // 11 LOW
  trains[TRAIN_58].speeds[26].curved_vel = 37543;
  trains[TRAIN_58].speeds[26].stopping_distance = 550;
  trains[TRAIN_58].speeds[27].straight_vel = 45551; // 12 LOW
  trains[TRAIN_58].speeds[27].curved_vel = 44540;
  trains[TRAIN_58].speeds[27].stopping_distance = 754;
  trains[TRAIN_58].speeds[28].straight_vel = 52030; // 13 LOW
  trains[TRAIN_58].speeds[28].curved_vel = 51056;
  trains[TRAIN_58].speeds[28].stopping_distance = 916;
  trains[TRAIN_58].speeds[29].straight_vel = 53344; // 14 LOW
  trains[TRAIN_58].speeds[29].curved_vel = 52635;
  trains[TRAIN_58].speeds[29].stopping_distance = 1188;

}

void init_switches( int *switch_states, int num_switches ) {
  switch_states[SW1] = SW_CURVED;
  switch_states[SW2] = SW_CURVED;
  switch_states[SW3] = SW_CURVED;
  switch_states[SW4] = SW_CURVED;
  switch_states[SW5] = SW_CURVED;
  switch_states[SW6] = SW_CURVED;
  switch_states[SW7] = SW_CURVED;
  switch_states[SW8] = SW_CURVED;
  switch_states[SW9] = SW_CURVED;
  switch_states[SW10] = SW_CURVED;
  switch_states[SW11] = SW_CURVED;
  switch_states[SW12] = SW_CURVED;
  switch_states[SW13] = SW_CURVED;
  switch_states[SW14] = SW_CURVED;
  switch_states[SW15] = SW_CURVED;
  switch_states[SW16] = SW_CURVED;
  switch_states[SW17] = SW_CURVED;
  switch_states[SW18] = SW_CURVED;
  switch_states[SW153] = SW_STRAIGHT;
  switch_states[SW154] = SW_CURVED;
  switch_states[SW155] = SW_STRAIGHT;
  switch_states[SW156] = SW_CURVED;
}

void update_velocity( train_state_t *train, int cur_time, int prev_time, int dist ) {
  int new_vel = ( dist * 10000 ) / ( cur_time - prev_time );
  (train->speeds[train->cur_speed]).straight_vel = 
    ( (80 * (train->speeds[train->cur_speed]).straight_vel ) + ( 20 * new_vel ) ) / 100;
}

// TODO: WILSON
int get_next_sensor( struct _train_state_ *train, int *dist_to_next_sensor ) {
  return 13;
}

