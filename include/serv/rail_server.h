#ifndef __RAIL_SERVER_H__
#define __RAIL_SERVER_H__

//NOTE; forward declare the structs used by the rail_server
//
struct _sensor_data_; 
struct _rail_cmds_;
struct _train_state;

typedef union _content_ {
  struct _sensor_data_* sensor_data;
  struct _rail_cmds_  * rail_cmds;
  struct _train_state_* train_state;
  void * nullptr;
} content_t;

typedef struct _rail_msg_ {
  
  enum {
    SENSOR_DATA = 0,
    CALIBRATION,
    RAIL_CMDS,
    TRAIN_EXE_READY,
    SWITCH_EXE_READY,
    TRAIN_DELAY_TIMEOUT,
  } request_type;

  content_t to_server_content;
  content_t from_server_content;
} rail_msg_t;


#endif
