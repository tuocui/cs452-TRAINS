/* Helper shit */

#include "util.h"
#include "tools.h"
#include "io.h"
#include "syscall.h"
#include "clock_server.h"

void idle_percent_task( ) {
  int cumulative_pct;
  int recent_pct;
  Printf( 
    COM2,
    "\0337\033[1;40HIdle (cumulative):   , (recent):   \0338"
  );
  FOREVER {
    cumulative_pct = Idle_pct_cumulative( );
    recent_pct = Idle_pct_recent( );
    // Print time
    Printf( 
      COM2,
      "\0337\033[1;58H%d%% \033[1;72H%d%% \0338",
      cumulative_pct,
      recent_pct
    );
    Delay( 100 );
  }
  Exit( );
}
