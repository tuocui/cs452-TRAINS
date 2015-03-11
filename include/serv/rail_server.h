#ifndef __RAIL_SERVER_H__
#define __RAIL_SERVER_H__

//NOTE; forward declare the structs used by the rail_server
//
struct _sensor_data_; 
struct _rail_cmds_;
struct _train_state;

#define WEIGHT_PREV 80
#define WEIGHT_NEW 20
#define SENSOR_WORKER_MAX 4
#define INIT_SPEED 2

typedef union _content_ {
  struct _sensor_data_* sensor_data;
  struct _rail_cmds_  * rail_cmds;
  struct _train_state_* train_state;
  struct _sensor_args_ *sensor_args;
  void * nullptr;
} content_t;

typedef struct _sensor_args_ {
  int sensor_num;
  struct _train_state_ *trains;
} sensor_args_t;

typedef struct _train_cmd_args_ {
  int cmd;
  int speed_num;
  int delay_time;
  int dest;
  int mm_past_dest;
} train_cmd_args_t;

typedef struct _switch_cmd_args_ {
  int state;
  int delay_time;
  int *switch_states;
} switch_cmd_args_t;

typedef struct _update_trains_args_ {
  struct _train_state_ *trains;
} update_train_args_t;

typedef struct _rail_msg_ {
  enum {
    SENSOR_DATA = 0,
    USER_INPUT,
    RAIL_CMDS,
    TRAIN_EXE_READY,
    SWITCH_EXE_READY,
    TRAIN_DELAY_TIMEOUT,
    TIMER_READY,
    SENSOR_WORKER_READY,
  } request_type;

  enum {
    PROCESS_SENSOR_WORK,
  } response_type;

  content_t to_server_content;
  //FIXME: remove from_server_content;
  content_t from_server_content;
} rail_msg_t;

void rail_server( );

#endif
