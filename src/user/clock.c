#include "tools.h"
#include "clock.h"
#include "io.h"
#include "syscall.h"
#include "clock_server.h"

void clock_user_task( ) {
  int cur_time;
  int time_in_min;
  int time_in_sec;
  int time_in_ds;
  FOREVER {
    cur_time = Delay( 10 );
    time_in_ds = cur_time / 10; 
    time_in_min = time_in_ds / (60 * 10);
    time_in_sec = ( time_in_ds / 10 ) % 60;
    time_in_ds = time_in_ds % 10;
    // Print time
    //Printf( 
    //  COM2,
    //  "\0337\033[HTime: %d min, %d sec, %d dsec\0338", 
    //  time_in_min,
    //  time_in_sec,
    //  time_in_ds
    //);
  }
  Exit( );
}
