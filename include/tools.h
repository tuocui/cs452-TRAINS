/*
 * tools.h
 */
#ifndef __TOOLS_H__
#define __TOOLS_H__

#include "bwio.h"
#include "global.h"

#define FOREVER for( ; ; )

#define assert( cond, ... )                                      \
  do {                                                           \
  if( !( cond ) )                                                \
    bwprintf(COM2,"Assert failed (%s:%d):\t%s\n\r\n\r",__FILE__, \
        __LINE__, ## __VA_ARGS__);                               \
  } while( 0 )

/* to enable debug print
 * call nmake with -d; ./nmake -d
 * Please do not change the default DEBUG value
 */
#ifndef DEBUG
#define DEBUG 0
#endif
#define debug( fmt, ... )                                     \
  do {                                                        \
    if( DEBUG ) {                                             \
      bwprintf(COM2, "DEBUG (%s: %d):\t"#fmt"\n\r", __FILE__, \
          __LINE__, ## __VA_ARGS__ );                         \
    }                                                         \
  } while( 0 )

#define min(a, b)              \
   ( {__typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
      _a < _b ? _a : _b; } )


#endif
