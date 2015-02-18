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
  
  COM1_out_msg_t msg;
  int msg_size = sizeof(msg);
  msg.request_type = CM1_OUT_READY;
  msg.msg_val = NULL;
  msg.msg_len = 0;
  char rpl;
  int errno;

  FOREVER {
    debug( "Before await" );
    errno = AwaitEvent( COM1_OUT_IND );
    assert(0, errno >= 0, "ERROR: interrupt eventid is incorrect" );
    debug( "Notifier finished awaiting" );

    Send( com1_out_server_tid, (char*)&msg, msg_size, &rpl, 1 );
    *((int *)( UART1_BASE + UART_DATA_OFFSET )) = rpl;
  }
}

void COM1_In_Notifier( ) {
  int com1_in_server_tid = MyParentTid( );

  COM1_in_msg_t msg;
  int msg_size = sizeof(msg);
  msg.request_type = CM1_IN_READY;
  char rpl;
  char c;

  FOREVER {
    c = (char) AwaitEvent( COM1_IN_IND );
    msg.val = c;
    Send( com1_in_server_tid, (char *)&msg, msg_size, &rpl, 0 );
  }
}

void COM2_Out_Notifier( ) {
  int com2_out_server_tid = MyParentTid( );
  
  COM1_out_msg_t msg;
  int msg_size = sizeof(msg);
  msg.request_type = CM1_OUT_READY;
  msg.msg_val = NULL;
  msg.msg_len = 0;
  char rpl;
  int errno;

  FOREVER {
    debug( "Before await" );
    errno = AwaitEvent( COM2_OUT_IND );
    assert(0, errno >= 0, "ERROR: interrupt eventid is incorrect" );
    debug( "Notifier finished awaiting" );

    Send( com2_out_server_tid, (char*)&msg, msg_size, &rpl, 1 );
    *((int *)( UART2_BASE + UART_DATA_OFFSET )) = rpl;
    debug( "after putc on UART2 ");
  }
}

void COM2_In_Notifier( ) {
  int com2_in_server_tid = MyParentTid( );

  COM1_in_msg_t msg;
  int msg_size = sizeof(msg);
  msg.request_type = CM1_IN_READY;
  char rpl;

  FOREVER {
    msg.val = (char)AwaitEvent( COM2_IN_IND );
    Send( com2_in_server_tid, (char *)&msg, msg_size, &rpl, 0 );
  }
}
void COM1_Out_Server( ) {
  if( RegisterAs( (char *)COM1_OUT_SERVER ) == -1) {
    bwputstr( COM2, "ERROR: failed to register COM1 OUTPUT server, aborting." );
    Exit( );
  }

  enable_uart( COM1 );
  int client_tid;
  COM1_out_msg_t msg;
  int msg_size = sizeof(msg);
  int notifier_ready = 0;
  int com1_out_cur_ind = 0;
  int com1_out_print_ind = 0;
  char com1_out_buf[OUT_BUF_SIZE];
  char c;
  char *client_msg;
  int i;
  int notifier_tid = Create( 2, &COM1_Out_Notifier );
  debug( "com1_out - notifier_tid: %d, server_tid: %d", notifier_tid, MyTid( ) );
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
      Reply( client_tid, client_msg, 0 );
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

void COM1_In_Server( ) {
  if( RegisterAs( (char *)COM1_IN_SERVER ) == -1) {
    bwputstr( COM2, "ERROR: failed to register COM1 INPUT server, aborting." );
    Exit( );
  }

  int client_tid;
  COM1_in_msg_t msg;
  char c_rpl = 'a';
  int msg_size = sizeof(msg);
  int com1_in_cur_ind = 0;
  int com1_in_print_ind = 0;
  int client_q_cur_ind = 0;
  int client_q_tail_ind = 0;
  char com1_in_buf[OUT_BUF_SIZE];
  int client_q[TD_MAX];
  int notifier_tid = Create( 1, &COM1_In_Notifier );
  debug( "com1_in - notifier_tid: %d, server_tid: %d", notifier_tid, MyTid( ) );
  FOREVER {
    Receive( &client_tid, (char *)&msg, msg_size );
    switch( msg.request_type ) {
    case CM1_IN_READY:
      Reply( client_tid, &c_rpl, 0 );
      // Add char to buffer
      com1_in_buf[com1_in_cur_ind] = msg.val;
      debug( "received char from uart: char:%x\r\n", com1_in_buf[com1_in_cur_ind] );
      com1_in_cur_ind = ( com1_in_cur_ind + 1 ) % OUT_BUF_SIZE;
      break;
    case CM1_GET:
      // Add client to queue
      client_q[client_q_cur_ind] = client_tid;
      client_q_cur_ind = ( client_q_cur_ind + 1 ) % TD_MAX;
      break;
    default:
      break;
    }

    if( com1_in_cur_ind != com1_in_print_ind && client_q_cur_ind != client_q_tail_ind ) {
      c_rpl = com1_in_buf[com1_in_print_ind];
      com1_in_print_ind = ( com1_in_print_ind + 1 ) % OUT_BUF_SIZE;
      client_tid = client_q[client_q_tail_ind];
      client_q_tail_ind = ( client_q_tail_ind + 1 ) % TD_MAX;
      //bwprintf( COM2, "sent: %x\r\n", c_rpl );
      Reply( client_tid, &c_rpl, 1 );
    }
  }
  Exit( );
}

void COM2_Out_Server( ) {
  if( RegisterAs( (char *)COM2_OUT_SERVER ) == -1) {
    Exit( );
  }

  int client_tid;
  COM1_out_msg_t msg;
  int msg_size = sizeof(msg);
  int notifier_ready = 0;
  int com2_out_cur_ind = 0;
  int com2_out_print_ind = 0;
  char com2_out_buf[OUT_BUF_SIZE];
  char c;
  char *client_msg;
  int i;
  int notifier_tid = Create( 2, &COM2_Out_Notifier );
  debug( "com2_out - notifier_tid: %d, server_tid: %d", notifier_tid, MyTid( ) );
  FOREVER {
    Receive( &client_tid, (char *)&msg, msg_size );
    switch( msg.request_type ) {
    case CM1_OUT_READY:
      notifier_ready = 1;
      break;
    case CM1_PUT:
      client_msg = msg.msg_val;
      for( i = 0; i < msg.msg_len; ++i ) {
        com2_out_buf[com2_out_cur_ind] = client_msg[i];
        com2_out_cur_ind = ( com2_out_cur_ind + 1 ) % OUT_BUF_SIZE;
      }
      Reply( client_tid, client_msg, 0 );
      break;
    default:
      break;
    }

    if( notifier_ready && com2_out_print_ind != com2_out_cur_ind ) {
      c = com2_out_buf[com2_out_print_ind];
      com2_out_print_ind = ( com2_out_print_ind + 1 ) % OUT_BUF_SIZE;
      notifier_ready = 0;
      Reply( notifier_tid, &c, 1 );
      //bwprintf( COM2, "sent: %c\r\n", c );
    }
  }
  Exit( );
}

void COM2_In_Server( ) {
  if( RegisterAs( (char *)COM2_IN_SERVER ) == -1) {
    bwputstr( COM2, "ERROR: failed to register COM1 INPUT server, aborting." );
    Exit( );
  }

  int client_tid;
  COM1_in_msg_t msg;
  char c_rpl = 'a';
  int msg_size = sizeof(msg);
  int com2_in_cur_ind = 0;
  int com2_in_print_ind = 0;
  int client_q_cur_ind = 0;
  int client_q_tail_ind = 0;
  char com2_in_buf[OUT_BUF_SIZE];
  int client_q[TD_MAX];
  int notifier_tid = Create( 1, &COM2_In_Notifier );
  debug( "com2_in - notifier_tid: %d, server_tid: %d", notifier_tid, MyTid( ) );
  FOREVER {
    Receive( &client_tid, (char *)&msg, msg_size );
    switch( msg.request_type ) {
    case CM1_IN_READY:
      Reply( client_tid, &c_rpl, 0 );
      // Add char to buffer
      com2_in_buf[com2_in_cur_ind] = msg.val;
      debug( "received char from uart: char:%x\r\n", com2_in_buf[com2_in_cur_ind] );
      com2_in_cur_ind = ( com2_in_cur_ind + 1 ) % OUT_BUF_SIZE;
      break;
    case CM1_GET:
      // Add client to queue
      client_q[client_q_cur_ind] = client_tid;
      client_q_cur_ind = ( client_q_cur_ind + 1 ) % TD_MAX;
      break;
    default:
      break;
    }

    if( com2_in_cur_ind != com2_in_print_ind && client_q_cur_ind != client_q_tail_ind ) {
      c_rpl = com2_in_buf[com2_in_print_ind];
      com2_in_print_ind = ( com2_in_print_ind + 1 ) % OUT_BUF_SIZE;
      client_tid = client_q[client_q_tail_ind];
      client_q_tail_ind = ( client_q_tail_ind + 1 ) % TD_MAX;
      //bwprintf( COM2, "sent: %x\r\n", c_rpl );
      Reply( client_tid, &c_rpl, 1 );
    }
  }
  Exit( );
}

int Putc( int channel, char ch ) {
  return Putstr( channel, &ch, 1 );
  return 0;
}

int Putstr( int channel, char *msg, int msg_len ) {
  COM1_out_msg_t com_msg;
  int com_msg_len = sizeof(com_msg);
  char rtn;
  com_msg.request_type = CM1_PUT;
  com_msg.msg_val = msg;
  com_msg.msg_len = msg_len;

  switch( channel ) {
  case COM1: {
    int com_out_server_tid = WhoIs( (char *)COM1_OUT_SERVER );
    Send( com_out_server_tid, (char *)&com_msg, com_msg_len, &rtn, 0 );
    break;
  }
  case COM2: {
    int com_out_server_tid = WhoIs( (char *)COM2_OUT_SERVER );
    Send( com_out_server_tid, (char *)&com_msg, com_msg_len, &rtn, 0 );
    break;
  }
  default:
    return -1;
    break;
  }

  return 0;
}

int Getc( int channel ) {
  COM1_in_msg_t com_msg;
  int com_msg_len = sizeof(com_msg);
  char rtn;
  com_msg.request_type = CM1_GET;
  com_msg.val = 0;

  switch( channel ) {
  case COM1:
  {
    int com_in_server_tid = WhoIs( (char *)COM1_IN_SERVER );
    Send( com_in_server_tid, (char *)&com_msg, com_msg_len, &rtn, 1 );
    return (int) rtn;
    break;
  }
  case COM2: {
    int com_in_server_tid = WhoIs( (char *)COM2_IN_SERVER );
    Send( com_in_server_tid, (char *)&com_msg, com_msg_len, &rtn, 1 );
    return (int) rtn;
    break;
  }
  default:
    return -1;
    break;
  }
  return 0;
}

int pputc( char c, char *buf, int *ind ) {
  if( *ind >= PRINTF_MAX_SIZE ) {
    return -1;
  }
  buf[(*ind)] = c;
  ++(*ind);
  return 0;
}

char pc2x( char ch ) {
  if ( (ch <= 9) ) return '0' + ch;
  return 'a' + ch - 10;
}

int pputx( char c, char *outbuf, int *ind ) {
  char chh, chl;

  chh = pc2x( c / 16 );
  chl = pc2x( c % 16 );
  pputc( chh, outbuf, ind );
  return pputc( chl, outbuf, ind );
}

int pputr( int channel, unsigned int reg, char *outbuf, int *ind ) {
  int byte;
  char *ch = (char *) &reg;

  for( byte = 3; byte >= 0; byte-- ) pputx( ch[byte], outbuf, ind );
  char c = ' ';
  return pputc( c, outbuf, ind );
}

void pputw( int channel, int n, char fc, char *bf, char *outbuf, int *ind ) {
  char ch;
  char *p = bf;

  while( *p++ && n > 0 ) n--;
  while( n-- > 0 ) pputc( fc, outbuf, ind );
  while( ( ch = *bf++ ) ) pputc( ch, outbuf, ind );
}

int pa2d( char ch ) {
  if( ch >= '0' && ch <= '9' ) return ch - '0';
  if( ch >= 'a' && ch <= 'f' ) return ch - 'a' + 10;
  if( ch >= 'A' && ch <= 'F' ) return ch - 'A' + 10;
  return -1;
}

char pa2i( char ch, char **src, int base, int *nump ) {
  int num, digit;
  char *p;

  p = *src; num = 0;
  while( ( digit = pa2d( ch ) ) >= 0 ) {
    if ( digit > base ) break;
    num = num*base + digit;
    ch = *p++;
  }
  *src = p; *nump = num;
  return ch;
}

void pui2a( unsigned int num, unsigned int base, char *bf ) {
  int n = 0;
  int dgt;
  unsigned int d = 1;

  while( (num / d) >= base ) d *= base;
  while( d != 0 ) {
    dgt = num / d;
    num %= d;
    d /= base;
    if( n || dgt > 0 || d == 0 ) {
      *bf++ = dgt + ( dgt < 10 ? '0' : 'a' - 10 );
      ++n;
    }
  }
  *bf = 0;
}

void pi2a( int num, char *bf ) {
  if( num < 0 ) {
    num = -num;
    *bf++ = '-';
  }
  pui2a( num, 10, bf );
}

/*int pputstr( int channel, char *str ) {
  while( *str ) {
    if( pputc( channel, *str ) < 0 ) return -1;
    str++;
  }
  return 0;
}*/

void pformat ( int channel, int *buf_size, char *outbuf, char *fmt, va_list va ) {
  char bf[12];
  char ch, lz;
  int w;
  int ind = 0;

  while ( ( ch = *(fmt++) ) ) {
    if ( ch != '%' ) {
      pputc( ch, outbuf, &ind );
    }
    else {
      lz = 0; w = 0;
      ch = *(fmt++);
      switch ( ch ) {
      case '0':
        lz = 1; ch = *(fmt++);
        break;
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        ch = pa2i( ch, &fmt, 10, &w );
        break;
      }
      switch( ch ) {
      case 0: return;
      case 'c':
        pputc( va_arg( va, char ), outbuf, &ind );
        break;
      case 's':
        pputw( channel, w, 0, va_arg( va, char* ), outbuf, &ind );
        break;
      case 'u':
        pui2a( va_arg( va, unsigned int ), 10, bf );
        pputw( channel, w, lz, bf, outbuf, &ind );
        break;
      case 'd':
        pi2a( va_arg( va, int ), bf );
        pputw( channel, w, lz, bf, outbuf, &ind );
        break;
      case 'x':
        pui2a( va_arg( va, unsigned int ), 16, bf );
        pputw( channel, w, lz, bf, outbuf, &ind );
        break;
      case '%':
        pputc( ch, outbuf, &ind );
        break;
      }
    }
  }
  *buf_size = ind;
  return;
}

void Printf( int channel, char *fmt, ... ) {
  va_list va;
  va_start(va,fmt);
  int len = 0;
  char outbuf[PRINTF_MAX_SIZE];
  pformat( channel, &len, outbuf, fmt, va );
  Putstr( channel, outbuf, len );
  va_end(va);
}

