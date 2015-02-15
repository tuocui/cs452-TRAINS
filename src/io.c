#include <tools.h>
#include "io.h"

void COM1_Out_Notifier( ) {
  // AwaitEvent, in AwaitEvent, turn on interrupt, then turn off
  // Send to server, receive char as reply
  // Stick char in UART
}
void COM1_Out_Server( ) {
  if( RegisterAs( (char *)COM1_OUT_SERVER ) == -1) {
    bwputstr( COM2, "ERROR: failed to register COM1 OUTPUT server, aborting." );
    Exit( );
  }

  int notifier_tid = Create( 1, &COM1_Out_Notifier );
  int client_tid;
  COM1_msg_t msg;
  COM1_msg_t rpl;
  int msg_size = sizeof(msg);
  debug( "com1_out - notifier_tid: %d, server_tid: %d", notifier_tid, MyTid( ) );
  FOREVER {
    Receive( &client_tid, (char *)&msg, msg_size );
    switch( msg.type ) {
    case CM1_OUT_READY:
      break;
    case CM1_PUT:
      break;
    default:
      break;
    }
  }
  Exit( );
}

int Putc( int channel, char ch ) {
  return 0;
}

int Getc( int channel ) {
  return 0;
}

