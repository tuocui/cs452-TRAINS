#ifndef __SYSCALL_H__
#define __SYSCALL_H__

#include <global.h>


int Create( int priority, void (*code) ( ) );

int MyTid( );

int MyParentTid( );

void Pass( );

void Exit( );

int Send( int tid, char *msg, int msglen, char *reply, int replylen );

int Receive( int *tid, char *msg, int msglen );

int Reply( int tid, char *reply, int replylen );

#endif
