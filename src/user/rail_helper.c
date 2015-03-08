#include "io.h"
#include "tools.h"
#include "track.h"
#include "clock_server.h"
#include "syscall.h"
#include "rail_helper.h"
#include "calibration.h"
#include "track_node.h"

// time to switch branch * velocity of train + buffer + length of train
// Assume 250ms to switch switch
// returns distance in mm
int safe_distance_to_branch( train_t *train ) {
  int vel = (train->speeds[train->cur_speed]).straight_vel;
  return ( vel * SWITCH_TIME ) / 100000 + SWITCH_BUFFER + train->len
}

// stopping distance + buffer + length of train
// return distance in mm
int safe_distance_to_stop( train_t *train ) {
  return (train->speeds[train->cur_speed]).stopping_distance + STOP_BUFFER + train->len);
}

// returns in ms
int accel_time( int cur_speed, int prev_speed ) {
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

int get_mm_past_last_landmark( train_t *train, int cur_time ) {
  int speed_change_time = train->speed_change_time * 10;
  int prev_node_time = train->time_at_last_landmark * 10;
  int speed_finish_time = speed_change_time + accel_time( train->cur_speed, train->prev_speed );
  int cur_vel = train->speeds[train->cur_speed].straight_vel;
  if( speed_finish_time < prev_node_time ) {
    return ( ( cur_time - prev_node_time ) * cur_vel ) / 10000;
  } else if( speed_finish_time < cur_time ) {
    int prev_vel = train->speeds[train->prev_speed].straight_vel;
    return ( ( cur_time - speed_finish_time ) * cur_vel ) / 10000 + 
           ( ( speed_finish_time - prev_node_time ) * prev_vel ) / 10000 + 
           ( ( speed_finish_time - prev_node_time ) * ( cur_vel - prev_vel ) ) / 20000;
  } else {
    return ( ( cur_time - prev_node_time ) * prev_vel ) / 10000 + 
           ( ( cur_time - prev_node_time ) * ( cur_vel - prev_vel ) ) / 20000;
  }
}

// node is index
// returns time in ms
// TODO: Figure this out
// dist = distance from previous landmark it passed to the node
int time_to_node( train_t *train, int dist_to_node, track_node_t *track ) {
  int cur_node = train->prev_landmark;
  int prev_node_time = train->time_at_last_landmark * 10;
  int cur_time = Time( ) * 10;
  int speed_change_time = train->speed_change_time * 10;

  int accel_time = accel_time( train->cur_speed, train->prev_speed );
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
