#ifndef __CLOCK_SERVER__
#define __CLOCK_SERVER__

#define UPDATE_TIME 0x11
#define NOTIFIER_MAGIC 0x12

typedef struct clock_msg_t {
  int value;

  enum {
    CM_TIME,
    CM_DELAY,
    CM_DELAY_UNTIL,
    CM_UPDATE,
    CM_REPLY,
  } request_type ;

} clock_msg_t ;

typedef struct clock_client_t {

  unsigned int c_tid;

  unsigned int future_ticks;

  struct clock_client_t *next_client_q;

} clock_client_t ;


//TODO: make a msg struct

void clock_server_init( );

void clock_clients_init( );

#endif
