#ifndef __SYSCALL_H__
#define __SYSCALL_H__

#include <global.h>

#ifndef COM1
#define COM1	0
#define COM2	1
#endif

typedef struct COM1_msg_t {
  enum {
    CM1_OUT_READY,
    CM1_PUT,
  } request_type ;
  int val;
} COM1_msg_t;


void COM1_Out_Server( );
int Putc( int channel, char ch );
int Getc( int channel );

#endif
