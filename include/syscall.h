#ifndef __SYSCALL_H__
#define __SYSCALL_H__

#include <global.h>

int Create( int priority, void (*code) ( ) );

int MyTid( );

int MyParentTid( );

void Pass( );

void Exit( );

#endif
