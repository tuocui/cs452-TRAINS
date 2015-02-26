#ifndef __IO_H__
#define __IO_H__

#include "global.h"

#ifndef COM1
  #define COM1	0
  #define COM2	1
#endif

#define LOW_SPEED 2400
#define HIGH_SPEED 115200
#define OUT_BUF_SIZE 2048
#define PRINTF_MAX_SIZE 128
#define MAX_IN_SIZE 16
#define MAX_OUT_SIZE 8

typedef struct com_msg {
  enum {
    COM_IN_READY,
    COM_IN_REPLY,
    COM_IN_GET,

    COM_OUT_READY,
    COM_OUT_REPLY,
    COM_OUT_PUT,
  } request_type;
  char * msg_val;
  int msg_len;
} com_msg_t ;


void com1_out_server( );
void com1_in_server( );
void com2_out_server( );
void com2_in_server( );
int Putc( int channel, char ch );
int Putstr( int channel, char *msg, int msg_len );
int Getc( int channel );
void Printf( int channel, char *fmt, ... );
int enable_uart( int channel );
int enable_two_stop_bits( int channel );
int setfifo( int channel, int state );
int setspeed( int channel, int speed );
void wait_cycles( int cycles );

#endif
