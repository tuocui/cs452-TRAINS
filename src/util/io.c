#include <tools.h>
#include <ts7200.h>
#include "syscall.h"
#include "nameserver.h"
#include "io.h"


// Turn on UART, should only be used for COM1
int enable_uart( int channel ) {
	int *uart_base;
	switch( channel ){
	case COM1:
		uart_base = (int *) UART1_BASE;
		break;
	case COM2:
		uart_base = (int *) UART2_BASE;
		break;
	default:
		return -1;
		break;
	}
	int *uart_ctrl = uart_base + (UART_CTLR_OFFSET / 4);
	*uart_ctrl = *uart_ctrl | UARTEN_MASK;
	return 0;
}

// Enables the two stop bits
// Used to properly communicate with train controller
int enable_two_stop_bits( int channel ) {
	int *uart_base;
	switch( channel ){
	case COM1:
		uart_base = (int *) UART1_BASE;
		break;
	case COM2:
		uart_base = (int *) UART2_BASE;
		break;
	default:
		return -1;
		break;
	}
	int *uart_lcrh = uart_base + (UART_LCRH_OFFSET / 4);
	// also double check use 8-bit
	*uart_lcrh = *uart_lcrh | STP2_MASK | WLEN_MASK;
	return 0;
}

int setfifo( int channel, int state ) {
	int *line, buf;
	switch( channel ) {
	case COM1:
		line = (int *)( UART1_BASE + UART_LCRH_OFFSET );
	        break;
	case COM2:
	        line = (int *)( UART2_BASE + UART_LCRH_OFFSET );
	        break;
	default:
	        return -1;
	        break;
	}
	buf = *line;
	buf = state ? buf | FEN_MASK : buf & ~FEN_MASK;
	*line = buf;
	return 0;
}

// Set communication speed
int setspeed( int channel, int speed ) {
	int *high, *low;
	switch( channel ) {
	case COM1:
		high = (int *)( UART1_BASE + UART_LCRM_OFFSET );
		low = (int *)( UART1_BASE + UART_LCRL_OFFSET );
	        break;
	case COM2:
		high = (int *)( UART2_BASE + UART_LCRM_OFFSET );
		low = (int *)( UART2_BASE + UART_LCRL_OFFSET );
	        break;
	default:
	        return -1;
	        break;
	}
	switch( speed ) {
	case 115200:
		*high = 0x0;
		*low = 0x3;
		return 0;
	case 2400:
		*high = 0x0;
		*low = 0xbf; //voodoo magic
		return 0;
	default:
		return -1;
	}
}

void wait_cycles( int cycles ) {
	while( cycles > 0 ) cycles--;
}

void COM1_Out_Notifier( ) {
  int com1_out_server_tid = MyParentTid( );
  //TODO: this should be a level 1 assert
  
  COM1_msg_t msg;
  int msg_size = sizeof(msg);
  msg.request_type = CM1_OUT_READY;
  msg.msg_val = NULL;
  msg.msg_len = 0;
  char rpl;
  int errno;

  FOREVER {
    debug( "Before await" );
    errno = AwaitEvent( COM1_OUT_IND );
    assert(0, errno > 0, "ERROR: interrupt eventid is incorrect" );
    debug( "Notifier finished awaiting" );

    Send( com1_out_server_tid, (char*)&msg, msg_size, &rpl, 1 );
	  *((int *)( UART1_BASE + UART_DATA_OFFSET )) = rpl;
    // assert(1, rpl == (char)NOTIFIER_MAGIC );
  }
  // AwaitEvent, in AwaitEvent, turn on interrupt, then turn off
  // Send to server, receive char as reply
  // Stick char in UART
}

void COM1_Out_Server( ) {
  if( RegisterAs( (char *)COM1_OUT_SERVER ) == -1) {
    bwputstr( COM2, "ERROR: failed to register COM1 OUTPUT server, aborting." );
    Exit( );
  }

  enable_uart( COM1 );
	setspeed( COM1, LOW_SPEED );
	wait_cycles(1000);
	enable_two_stop_bits( COM1 );
	setfifo( COM1, OFF );

  int notifier_tid = Create( 1, &COM1_Out_Notifier );
  int client_tid;
  COM1_msg_t msg;
  int msg_size = sizeof(msg);
  int notifier_ready = 0;
	int com1_out_cur_ind = 0;
	int com1_out_print_ind = 0;
  debug( "com1_out - notifier_tid: %d, server_tid: %d", notifier_tid, MyTid( ) );
  char com1_out_buf[OUT_BUF_SIZE];
  char c;
  char *client_msg;
  int i;
  FOREVER {
    Receive( &client_tid, (char *)&msg, msg_size );
    switch( msg.request_type ) {
    case CM1_OUT_READY:
      notifier_ready = 1;
      break;
    case CM1_PUT:
      client_msg = msg.msg_val;
      for( i = 0; i < msg.msg_len; ++i ) {
        com1_out_buf[com1_out_cur_ind] = client_msg[i];
        com1_out_cur_ind = ( com1_out_cur_ind + 1 ) % OUT_BUF_SIZE;
      }
      Reply( client_tid, client_msg, 1 );
      break;
    default:
      break;
    }

    if( notifier_ready && com1_out_print_ind != com1_out_cur_ind ) {
      c = com1_out_buf[com1_out_print_ind];
      com1_out_print_ind = ( com1_out_print_ind + 1 ) % OUT_BUF_SIZE;
      notifier_ready = 0;
      Reply( notifier_tid, &c, 1 );
      //bwprintf( COM2, "sent: %c\r\n", c );
    }
  }
  Exit( );
}

int Putc( int channel, char ch ) {
  return 0;
}

int Putstr( int channel, char *msg, int msg_len ) {
	switch( channel ) {
	case COM1:
  {
    COM1_msg_t com1_msg;
    int com1_out_server_tid = WhoIs( (char *)COM1_OUT_SERVER );
    int com1_msg_len = sizeof(com1_msg);
    char rtn;
		com1_msg.request_type = CM1_PUT;
    com1_msg.msg_val = msg;
    com1_msg.msg_len = msg_len;
    Send( com1_out_server_tid, (char *)&com1_msg, com1_msg_len, &rtn, 1 );
		break;
  }
	case COM2:
		break;
	default:
		return -1;
		break;
	}
  return 0;
}

int Getc( int channel ) {
  return 0;
}

