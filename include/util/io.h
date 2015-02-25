#ifndef __IO_H__
#define __IO_H__

#include <global.h>

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

typedef struct COM_var_msg_t {
  enum {
    CM1_OUT_READY,
    CM1_PUT,
    CM1_OUT_REPLY,
  } request_type ;
  char *msg_val;
  int msg_len;
} COM1_var_msg_t;

typedef struct COM_const_msg_t {
  enum {
    CM1_IN_READY,
    CM1_GET,
    CM1_IN_REPLY,
  } request_type ;
  char val;
} COM1_const_msg_t;


void COM1_Out_Server( );
void COM1_In_Server( );
void COM2_Out_Server( );
void COM2_In_Server( );
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
