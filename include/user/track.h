#ifndef __TRACK_H__
#define __TRACK_H__

#include "global.h"

#define NUM_SENSOR_BYTES 10
#define NUM_RECENT_SENSORS 5
#define REQUEST_SENSOR 133
#define KILL_SWITCH 32
#define NUM_TRAINS 80
#define TRACK_START 96
#define TRACK_STOP 97
#define CURVED 34
#define STRAIGHT 33

int track_go( );

int track_stop( );

short set_train_speed( short train, short speed );

int update_switch_output( short switch_num, char state );

int set_switch( short switch_num, short c_s );

int kill_switch( );

void track_sensor_task( );

int initialize_track( );

#endif
