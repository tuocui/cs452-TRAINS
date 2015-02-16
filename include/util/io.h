#ifndef __IO_H__
#define __IO_H__

#include <global.h>

  #ifndef COM1
    #define COM1	0
    #define COM2	1
  #endif

#define OUT_BUF_SIZE 512
#define LOW_SPEED 2400
#define HIGH_SPEED 115200

typedef struct COM1_msg_t {
  enum {
    CM1_OUT_READY,
    CM1_PUT,
    CM1_REPLY,
  } request_type ;
  char *msg_val;
  int msg_len;
} COM1_msg_t;


void COM1_Out_Server( );
int Putc( int channel, char ch );
int Putstr( int channel, char *msg, int msg_len );
int Getc( int channel );

#endif
