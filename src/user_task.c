#include <tools.h>
#include <user_task.h>

void a1_user_task( ){
  unsigned int my_tid;
  unsigned int parent_tid;
  my_tid = MyTid( );
  parent_tid = MyParentTid( );
  debug( "My TID: %d, Parent TID: %d", my_tid, parent_tid );
  Pass( );
  debug( "My TID: %d, Parent TID: %d", my_tid, parent_tid );
  Exit( );
}

void first_user_task( ){
  int created_tid;
  created_tid = Create( 10, &a1_user_task );
  debug( "Created: %d, Priority: Lower than First", created_tid);
  created_tid = Create( 1, &a1_user_task );
  debug( "Created: %d, Priority: Higher than First", created_tid);
  created_tid = Create( 1, &a1_user_task );
  debug( "Created: %d, Priority: Higher than First", created_tid);
  created_tid = Create( 10, &a1_user_task );
  debug( "Created: %d, Priority: Lower than First", created_tid);
  debug( "First: exiting");
  Exit( );
}
