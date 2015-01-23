#include <tools.h>
#include <user_task.h>
#include "syscall.h"

#undef A1
#ifdef A1
void a1_user_task( ){
  unsigned int my_tid;
  unsigned int parent_tid;
  my_tid = MyTid( );
  parent_tid = MyParentTid( );
  bwprintf( COM2, "My TID: %d, Parent TID: %d\n\r", TID_IDX(my_tid),
      TID_IDX(parent_tid) );
  Pass( );
  bwprintf( COM2, "My TID: %d, Parent TID: %d\n\r", TID_IDX(my_tid),
      TID_IDX(parent_tid) );
  Exit( );
}
#endif /* A1 */

//TODO remove testing struct
struct Server {
  int a;
  int b;
};
#ifdef A2
void a2_user_task( ) {
  
  struct Server server_s;
  struct Server server_r;
  char * msg;
  char * reply;
  
  msg = (char*)&server_s;
  reply = (char*)&server_r;
  debug("tid: %d, msg: %x, msglen: %d, reply: %x, replylen: %d", 5, msg, 6, reply, 7);
  Send( 5, msg, 6, reply, 7);
}
#endif /* A2 */

void first_user_task( ){
#ifdef A1
  int created_tid;
  created_tid = Create( 10, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Lower  than First\n\r", TID_IDX(created_tid));
  created_tid = Create( 1, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Higher than First\n\r", TID_IDX(created_tid));
  created_tid = Create( 1, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Higher than First\n\r", TID_IDX(created_tid));
  created_tid = Create( 10, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Lower  than First\n\r", TID_IDX(created_tid));
#endif /* A1 */

#ifdef A2
  a2_user_task( );
#endif /* A2 */

  bwprintf( COM2, "First: exiting\n\r");
  Exit( );
}


