 /*
 * kern.c
 */

#include <bwio.h>
//#include <ts7200.h>


void exit_kernel(int i) {
  bwprintf( COM2, "Hellow %d.\n\r", i);
}

