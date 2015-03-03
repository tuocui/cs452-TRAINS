#include "tools.h"
#include "io.h"
#include "syscall.h"
#include "track.h"
#include "screen.h"
#include "clock_server.h"

int output_invalid( ) {
  Putstr( COM2, "\0337\033[1A\033[2K\rInvalid command\0338", 28 );
  return 0;
}

// Really simple parsing command, look at first 3 chars only
int get_cmd( char *cmd_buffer ) {
  char cmd1 = cmd_buffer[0];
  char cmd2 = cmd_buffer[1];
  char cmd3 = cmd_buffer[2];
  if ( cmd1 == 'q' && ( cmd2 == ' ' || cmd2 == 0 ) ) {
    return QUIT_CMD;  
  }
  if ( cmd1 == 'g' && ( cmd2 == ' ' || cmd2 == 0 ) ) {
    return GO_CMD;  
  }
  if ( cmd1 == 'k' && ( cmd2 == ' ' || cmd2 == 0 ) ) {
    return KILL_CMD;  
  }
  if ( cmd3 != ' ' ) {
    return -1;
  }
  if ( cmd1 == 't' && cmd2 == 'r' ) {
    return MOVE_CMD;  
  }
  if ( cmd1 == 'r' && cmd2 == 'v' ) {
    return REV_CMD;  
  }
  if ( cmd1 == 's' && cmd2 == 'w' ) {
    return SWITCH_CMD;  
  }
  return -1;
}

short parse_short( char *cmd_buffer, int *buf_ind_ptr ) {
  short num_so_far = 0;
  while ( cmd_buffer[*buf_ind_ptr] != ' ' && cmd_buffer[*buf_ind_ptr] != 0 ) {
    if ( !( cmd_buffer[*buf_ind_ptr] >= 48 && 
         cmd_buffer[*buf_ind_ptr] <= 57 ) ) {
      return -1;    
    }
    num_so_far = num_so_far * 10;
    num_so_far += ( (short) cmd_buffer[*buf_ind_ptr] - 48 );
    ++(*buf_ind_ptr);
  }
  return num_so_far;
}

short parse_curve_straight( char *cmd_buffer, int *buf_ind_ptr ) {
  char c = cmd_buffer[*buf_ind_ptr];
  if ( c == 'S' ) {
    return 33;  
  }
  if ( c == 'C' ) {
    return 34;  
  }
  return -1;
}

int handle_move( char *cmd_buffer, short *train_speeds ) {
  int buf_ind = 3;
  short train = parse_short( cmd_buffer, &buf_ind );
  ++buf_ind;
  short speed = parse_short( cmd_buffer, &buf_ind );

  if ( train > 0 && speed >= 0 ){
    train_speeds[train] = set_train_speed( train, speed);
    Printf( COM2, "\0337\033[1A\033[2K\rTrain %d set to %d\0338", train, speed );
  } else {
    output_invalid( );
    return -1;
  }
  return 0;
}

int handle_rev( char *cmd_buffer, short *train_speeds ) {
  int buf_ind = 3;
  short train = parse_short( cmd_buffer, &buf_ind );
  short prev_speed;
  if ( train <= 0 ){
    output_invalid( );
    return -1;
  }
  prev_speed = train_speeds[train];
  Printf( COM2, "\0337\033[1A\033[2K\rTrain %d reversed\0338", train );
  set_train_speed( train, 0 );
  Delay( 350 );
  set_train_speed( train, 15 );
  set_train_speed( train, prev_speed );
  return 0;
}

int handle_switch( char *cmd_buffer ) {
  int buf_ind = 3;
  short switch_num = parse_short( cmd_buffer, &buf_ind );
  ++buf_ind;
  short c_s = parse_curve_straight( cmd_buffer, &buf_ind );
  char c_s_c;
  if ( switch_num <= 0 ||
       c_s <= 0 ||
       ( switch_num > 18 && !( switch_num >= 153 && switch_num <= 156 ) ) ) {
    output_invalid( );
    return -1;  
  }
  switch( c_s ) {
  case STRAIGHT:
    c_s_c = 'S';
    break;
  case CURVED:
    c_s_c = 'C';
    break;
  default:
    c_s_c = '/';
    break;
  }
  Printf( COM2, "\0337\033[1A\033[2K\rSwitch %d set to %c\0338", switch_num, c_s_c );
  set_switch( switch_num, c_s );
  return 0;
}

int handle_go( ) {
  track_go( );
  Putstr( COM2, "\0337\033[1A\033[2K\rTrack ON\0338", 21 );
  return 0;
}

int handle_kill( ) {
  track_stop( );
  Putstr( COM2, "\0337\033[1A\033[2K\rTrack OFF\0338", 22 );
  return 0;
}

int process_buffer( char *cmd_buffer, short *train_speeds ) {
  int cmd = get_cmd( cmd_buffer );

  switch( cmd ) {
  case MOVE_CMD:
    handle_move( cmd_buffer, train_speeds );
    break;
  case REV_CMD:
    handle_rev( cmd_buffer, train_speeds );
    break;
  case SWITCH_CMD:
    handle_switch( cmd_buffer );
    break;
  case GO_CMD:
    handle_go( );
    break;
  case KILL_CMD:
    handle_kill( );
    break;
  case QUIT_CMD:
    return QUIT_CMD;
    break;
  default:
    output_invalid( );
    break;
  }
  return 0;
}

void parse_user_input( ) {
  char c;
  int cmd_ind = 0;
  char cmd_buffer[CMD_BUF_SIZE];
  int status;
  short train_speeds[NUM_TRAINS];
  int i;
  for( i = 0; i < NUM_TRAINS; ++i ) {
    train_speeds[i] = 0;
  }

  FOREVER {
    c = (char)Getc( COM2 );
    // backspace char?
    if ( c == 8 && cmd_ind > 0 ) {
      --cmd_ind;
      cmd_buffer[cmd_ind] = 0;
      // Remove char from screen
      Putstr( COM2, "\033[1D \033[1D", 9 );
    }

    // enter char?
    if ( c == 13 ) {
      cmd_buffer[cmd_ind] = 0;
      cmd_ind = 0;
      Putstr( COM2, "\033[24;0H\033[2K\033[24;0H>", 19 );
      status = process_buffer( cmd_buffer, train_speeds );
      if( status == QUIT_CMD ) {
        Putstr( COM2, "\0337\033[1A\033[2K\rShutting down. Goodbye!\033[24;0H\033[2K", 45 );
        for( i = 25; i < NUM_TRAINS; ++i ) {
          train_speeds[i] = 0;
          set_train_speed( i, 0 );
        }
        Delay( 500 );
        Kill_the_system( 0xdeadbeef );
      }
    }

    // buffer too full?
    if ( cmd_ind >= 80 ) {
      continue;
    }

    // alphanumeric or space? Just throw it in the buffer
    if ( ( c >= 65 && c <= 90 ) ||
         ( c >= 97 && c <= 122 ) ||
         ( c >= 48 && c <= 57 ) ||
         ( c == 32 ) ) {
      cmd_buffer[cmd_ind] = c;
      ++cmd_ind;
      Putc( COM2, c );
    }
  }
}


