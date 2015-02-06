#include <tools.h>
#include <syscall.h>
#include <nameserver.h>

void reply_back( unsigned int tid, nameserver_msg_t *reply, int msg_len, int type, int val ) {
  reply->type = type;
  reply->val = val;
  Reply( tid, (char *)reply, msg_len );
}

void nameserver_main( ) {
  unsigned int jobs[SERVER_MAX];
  int i = 0;
  int reply_tid;
  int rtn_tid;
  nameserver_msg_t request;
  nameserver_msg_t reply;
  int msg_len = sizeof(request);
  int rcv_len;

  for( ; i < SERVER_MAX; ++i ) {
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
      if( !( request.val >= 0 && request.val < SERVER_MAX) ) {
        reply_back( reply_tid, &reply, msg_len, ERROR, INVALID_JOB );
      } else {
        // Register, and reply back
        jobs[request.val] = reply_tid;
        reply_back( reply_tid, &reply, msg_len, SUCCESS, 0 );
        debug( "Woo, a %d server registered with id: %d", request.type, jobs[request.type] );
      }
      break;
    case WHOIS:
      // Invalid job?
      if( !( request.val >= 0 && request.val < SERVER_MAX ) ) {
        reply_back( reply_tid, &reply, msg_len, ERROR, INVALID_JOB );
      } else {
        // Reply back with the tid
        rtn_tid = jobs[request.val];
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

// I'm currently just going to cast this to a char* when we pass it int
// and then cast back to an int here. There's gotta be a better way of doing it.
int RegisterAs( char *name ) {
  nameserver_msg_t request;
  nameserver_msg_t reply;
  request.type = REGISTER;
  request.val = (int) name;
  // Currently hard-coding the nameserver tid
  // TODO: Make the nameserver tid "global"
  int rtn = Send( 33, (char *)&request, sizeof(request), (char *)&reply, sizeof(reply) );
  // Nameserver doens't exist
  if( rtn < 0 ) {
    return -1;
  }
  return reply.val;
}

int WhoIs( char *name ) {
  nameserver_msg_t request;
  nameserver_msg_t reply;
  request.type = WHOIS;
  request.val = (int) name;
  int rtn = Send( 33, (char *)&request, sizeof(request), (char *)&reply, sizeof(reply) );
  // Nameserver doens't exist
  if( rtn < 0 ) {
    return -1;
  }
  return reply.val;
}

