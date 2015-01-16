/*
 * tools.h
 */
#ifndef __TOOLS_H__
#define __TOOLS_H__

#include "io.h"

#define FOREVER for( ; ; )


#define assert(cond) \
  if(!(cond)) { \
    bwprintf(COM2, "Assert failed (%s:%d)\n\r\n\r", __FILE__, __LINE__); }
#define assertm(cond, ...) \
  if(!(cond)) { \
    bwprintf(COM2,"Assert failed (%s:%d): %s\n\r\n\r",__FILE__,__LINE__,__VA_ARGS__);\
  }

/* to disable debug print, in your source file, do:
 * #undef DEBUG
 * #define DEBUG 0
 */
#define DEBUG 1
#define debug(fmt, ...) \
  if(DEBUG == 1) { \
    bwprintf(COM2, fmt, __VA_ARGS__);\
  }


//TODO: this function may have dest and source args mis-ordered, 
//  DO NOT USE IT BEFORE DOUBLE CHECKING 
//void memcpy(void *dest, const void *source, unsigned int num) {
//  int i = 0;
//// casting pointers
//  char *dest8 = (char *)dest;
//  char *source8 = (char *)source;
//  bwprintf(COM2, "Copying memory %d byte(s) at a time\n", sizeof(char));
//
//  for (i = 0; i < num; i++) {
//    // make sure destination doesnt overwrite source
//    if (&dest8[i] == source8) {
//      bwprintf(COM2, "destination array address overwrites source address\n");
//      return;
//    }
//  dest8[i] = source8[i];
//  }
//}

#endif
