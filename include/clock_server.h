#ifndef __CLOCK_SERVER__
#define __CLOCK_SERVER__

#define UPDATE_TIME 0x11
#define NOTIFIER_MAGIC 0x12
#define TICKS_TEN_MS 5080


/* timer stuff */ // TODO: shouldn't provide this interface 
void start_clock( int load_val );

inline int get_timer_val( );


/* clock server stuff */
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

typedef struct clock_client_msg_t {
  int delay_time;
  int num_delays;
} clock_client_msg_t;

void clock_server( );

int Delay( int ticks );

int DelayUntil( int ticks );

int Time( );

#endif /* __CLOCK_SERVER__ */
