/*
 * tools.h
 */
#ifndef __TOOLS_H__
#define __TOOLS_H__

#include "bwio.h"

#define FOREVER for( ; ; )


#define assert(cond, ...) \
  do {\
  if(!(cond))  \
    bwprintf(COM2,"Assert failed (%s: %s: %d): %s\n\r\n\r",__FILE__, __func__, __LINE__, ## __VA_ARGS__);\
  } while (0)

/* to enable debug print
 * call nmake with -d; ./nmake -d
 * Please do not change the default DEBUG value
 */
#ifndef DEBUG
#define DEBUG 0
#endif
#define debug(fmt, ...) \
  do { \
    if(DEBUG) { \
      bwprintf(COM2, "DEBUG (%s: %s: %d)\t", __FILE__, __func__, __LINE__);\
      bwprintf(COM2, fmt, ## __VA_ARGS__);\
      bwprintf(COM2, "\n\r");\
    } \
  } while(0)


#endif
