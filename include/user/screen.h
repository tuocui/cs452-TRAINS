#ifndef __SCREEN_H__
#define __SCREEN_H__

#include "global.h"

#define CMD_BUF_SIZE 80
#define MOVE_CMD 0
#define REV_CMD 1
#define SWITCH_CMD 2
#define GO_CMD 3
#define KILL_CMD 4
#define INIT_CMD 5
#define DEST_CMD 6
#define ACCEL_CMD 7
#define DECEL_CMD 8
#define CH_DIR_CMD 9
#define RSV_CMD 10
#define RAND_CMD 11
#define CTRL_CMD 12
#define PRIORITY_CMD 13
#define CLEAR_CMD 98
#define QUIT_CMD 99

#define TRAIN_CTRL 0
#define CMD_LINE 1

void parse_user_input( );
short parse_sensor_name( char *cmd_buffer, int *buf_ind_ptr );
short parse_short( char *cmd_buffer, int *buf_ind_ptr );

#endif
