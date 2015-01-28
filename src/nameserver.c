#include <tools.h>
#include <syscall.h>
#include <nameserver.h>

void reply_back( unsigned int tid, nameserver_msg_t *reply, int msg_len, int type, int val ) {
  reply->type = type;
  reply->val = val;
  Reply( tid, (char *)reply, msg_len );
}

void nameserver_main( ) {
  unsigned int jobs[NUM_JOBS];
  int i = 0;
  int reply_tid;
  int rtn_tid;
  nameserver_msg_t request;
  nameserver_msg_t reply;
  int msg_len = sizeof(request);
  int rcv_len;

  for( ; i < NUM_JOBS; ++i ) {
    jobs[i] = 0;
  }
  debug( "INITIALIZED NAMESERVER" );
  FOREVER {
    rcv_len = Receive( &reply_tid, (char *)&request, msg_len );
    if( rcv_len < msg_len ){
      reply_back( reply_tid, &reply, msg_len, ERROR, MESSAGE_TOO_SHORT );
      continue;
    }

    switch( request.type ) {
    case REGISTER:
      // Invalid job?
      if( !( request.type > 0 && request.type <= NUM_JOBS ) ) {
        reply_back( reply_tid, &reply, msg_len, ERROR, INVALID_JOB );
      } else {
        // Register, and reply back
        jobs[request.type] = reply_tid;
        reply_back( reply_tid, &reply, msg_len, SUCCESS, request.type );
      }
      break;
    case WHOIS:
      // Invalid job?
      if( !( request.type > 0 && request.type <= NUM_JOBS ) ) {
        reply_back( reply_tid, &reply, msg_len, ERROR, INVALID_JOB );
      } else {
        // Reply back with the tid
        rtn_tid = jobs[request.type];
        if( rtn_tid == 0 ) {
          reply_back( reply_tid, &reply, msg_len, ERROR, SERVER_NOT_FOUND );
        } else {
          reply_back( reply_tid, &reply, msg_len, SUCCESS, rtn_tid );
        }
      }
      break;
    default:
      reply_back( reply_tid, &reply, msg_len, ERROR, INVALID_REQUEST );
      break;
    }
  }

  Exit( );
}
