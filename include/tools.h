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

/* to disable debug print, in your source file, do:
 * #undef DEBUG
 * #define DEBUG 0
 * TODO: let preprocessor defines DEBUG 1 inside nmake
 */
#define DEBUG 0
#define debug(fmt, ...) \
  do { \
    if(DEBUG) { \
      bwprintf(COM2, "DEBUG (%s: %s: %d) ", __FILE__, __func__, __LINE__);\
      bwprintf(COM2, fmt, ## __VA_ARGS__);\
      bwprintf(COM2, "\n\r");\
  } while (0)



#endif
