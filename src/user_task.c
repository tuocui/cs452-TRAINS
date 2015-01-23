#include <tools.h>
#include <user_task.h>
#include "syscall.h"

#ifdef A1
void a1_user_task( ){
  unsigned int my_tid;
  unsigned int parent_tid;
  my_tid = MyTid( );
  parent_tid = MyParentTid( );
  bwprintf( COM2, "My TID: %d, Parent TID: %d\n\r", my_tid, parent_tid );
  Pass( );
  bwprintf( COM2, "My TID: %d, Parent TID: %d\n\r", my_tid, parent_tid );
  Exit( );
}
#endif /* A1 */


void first_user_task( ){
#ifdef A1
  int created_tid;
  created_tid = Create( 10, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Lower  than First\n\r", created_tid);
  created_tid = Create( 10, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Lower  than First\n\r", created_tid);
  created_tid = Create( 1, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Higher than First\n\r", created_tid);
  created_tid = Create( 1, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Higher than First\n\r", created_tid);
#endif /* A1 */

  bwprintf( COM2, "First: exiting\n\r");
  Exit( );
}
