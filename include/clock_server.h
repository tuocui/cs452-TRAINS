#ifndef __CLOCK_SERVER__
#define __CLOCK_SERVER__

#define UPDATE_TIME 0x11
#define NOTIFIER_MAGIC 0x12

typedef struct clock_msg_t {
  int request_type;
  int ticks;
}

typedef struct clock_client_t {

  unsigned int future_ticks;

  unsigned int c_tid;

  struct clock_client_t *next_client_q;
  struct clock_client_t *next_client_free;

  enum {
    CC_INACTIVE,
    CC_TIME,
    CC_DELAY,
    CC_DELAY_UNTIL,
  } request_type ;

} clock_client_t ;


//TODO: make a msg struct

void clock_server_init( );

void clock_clients_init( );

#endif
