#include <tools.h>
#include <user_task.h>
#include "syscall.h"

//TODO remove testing struct
struct Server {
  char a;
  char b;
};

void gen_user_task( ){
  struct Server server_r;
  int i = 0;
  int tid;
  char * msg;
  int msglen = sizeof(server_r);
  msg = (char*)&server_r;
  for( ; i < 5; ++i ){
    debug( "In gen_user_task, iter: %d", i );
    Pass( );
  }
  debug("tid: %x, msg: %x, msglen: %d", &tid, &server_r, msglen);
  int rtn = Receive( &tid, (char *) &server_r, msglen );
  debug("rtn: %d", rtn);
  debug("server_r.a: %c, server_r.b: %c", server_r.a, server_r.b);
  Exit( );
}

void user_send_task( ){
  struct Server server_s;
  struct Server server_reply;
  int my_tid = MyTid( );
  int receiver_tid = 33;
  server_s.a = (char) my_tid;
  server_s.b = (char) my_tid;
  debug(
    "tid: %d, msg: %x, msglen: %d, reply: %x, replylen: %d", 
    receiver_tid, &server_s,  sizeof(server_s), &server_reply,  sizeof(server_reply)
  );
  int rtn = Send( receiver_tid, (char *) &server_s,  sizeof(server_s), (char *) &server_reply,
      sizeof(server_reply));

  debug( "Send's rtn: %d; replymsg: %c%c", rtn,  server_reply.a, server_reply.b );
  
}

void user_receive_task( ){
  int sender_tid, rtn;
  struct Server server_r;
  struct Server server_reply;
  server_reply.a = 'f';
  server_reply.b = 'u';

  //int msglen = sizeof(server_r);
  debug(
    "tid: %d, msg: %x, msglen: %d", 
    &sender_tid, &server_r,  sizeof(server_r)
  );

  //rtn = Receive( &sender_tid, (char *) &server_r, msglen );
  //debug("rtn: %d", rtn);
  //debug("server_r.a: %d, server_r.b: %d", server_r.a, server_r.b);
  //rtn = Receive( &sender_tid, (char *) &server_r, msglen );
  //debug("rtn: %d", rtn);
  //debug("server_r.a: %d, server_r.b: %d", server_r.a, server_r.b);
  rtn = Reply( 31, (char*)&server_reply, sizeof(server_reply)+1 );
  debug("Reply's rtn: %d", rtn );
  Exit( );
}

#undef A1
//#undef A2
#ifdef A1
void a1_user_task( ){
  unsigned int my_tid;
  unsigned int parent_tid;
  my_tid = MyTid( );
  parent_tid = MyParentTid( );
  bwprintf( COM2, "My TID: %d, Parent TID: %d\n\r", my_tid, parent_tid );
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(my_tid), TID_GEN(my_tid));
  Pass( );
  bwprintf( COM2, "My TID: %d, Parent TID: %d\n\r", my_tid, parent_tid );
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(my_tid), TID_GEN(my_tid));
  Exit( );
}
#endif /* A1 */
#ifdef A2
void a2_user_task( ) {
  int receiver_tid = Create( 10, &user_receive_task );
  int sender_tid1 = Create( 1, &user_send_task );
  int sender_tid2 = Create( 1, &user_send_task );
  debug( "Receiver tid: %d, sender1 tid: %d, sender2 tid: %d", receiver_tid, sender_tid1, sender_tid2 );
  Exit( );
}
#endif /* A2 */

void first_user_task( ){
#ifdef A1
  int first_tid = MyTid( );
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(first_tid), TID_GEN(first_tid));

  int created_tid;
  created_tid = Create( 10, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Lower than First\n\r", created_tid);
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(created_tid), TID_GEN(created_tid));

  created_tid = Create( 10, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Lower than First\n\r", created_tid);
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(created_tid), TID_GEN(created_tid));

  created_tid = Create( 1, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Higher than First\n\r", created_tid);
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(created_tid), TID_GEN(created_tid));

  created_tid = Create( 1, &a1_user_task );
  bwprintf( COM2, "Created: %d, Priority: Higher than First\n\r", created_tid);
  debug( "TID_IDX: %d, TID_GEN: %d", TID_IDX(created_tid), TID_GEN(created_tid));
#endif /* A1 */

#ifdef A2
  a2_user_task( );
#endif /* A2 */

  bwprintf( COM2, "First: exiting\n\r");
  Exit( );
}


