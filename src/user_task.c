#include <tools.h>
#include <user_task.h>

void a1_user_task( ){
  unsigned int my_tid;
  unsigned int parent_tid;
  my_tid = MyTid( );
  parent_tid = MyParentTid( );
  bwprintf( COM2, "My TID: %d, Parent TID: %d\r\n", my_tid, parent_tid );
  Pass( );
  bwprintf( COM2, "My TID: %d, Parent TID: %d\r\n", my_tid, parent_tid );
  Exit( );
}

void first_user_task( ){
  int created_tid;
  created_tid = Create( 10, &a1_user_task );
  bwprintf(COM2, "Created: %d, Priority: Lower than First\r\n", created_tid);
  created_tid = Create( 1, &a1_user_task );
  bwprintf(COM2, "Created: %d, Priority: Higher than First\r\n", created_tid);
  created_tid = Create( 1, &a1_user_task );
  bwprintf(COM2, "Created: %d, Priority: Higher than First\r\n", created_tid);
  created_tid = Create( 10, &a1_user_task );
  bwprintf(COM2, "Created: %d, Priority: Lower than First\r\n", created_tid);
  bwputstr(COM2, "First: exiting\r\n");
  Exit( );
}
