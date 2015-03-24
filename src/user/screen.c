#include "tools.h"
#include "io.h"
#include "syscall.h"
#include "track.h"
#include "screen.h"
#include "clock_server.h"
#include "rail_server.h"
#include "rail_control.h"
#include "nameserver.h"

int output_invalid( ) {
  //Putstr( COM2, "\0337\033[1A\033[2K\rInvalid command\0338", 28 );
  Printf( COM2, "Invalid command\n\r" );
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
  if ( cmd1 == 'c' && cmd2 == 'l' && ( cmd3 == ' ' || cmd3 == 0 ) ) {
    return CLEAR_CMD;  
  }
  if ( cmd3 != ' ' ) {
    return -1;
  }
  if ( cmd1 == 't' && cmd2 == 'r' ) {
    return MOVE_CMD;  
  }
  if ( cmd1 == 't' && cmd2 == 'i' ) {
    return INIT_CMD;  
  }
  if ( cmd1 == 'a' && cmd2 == 'c' ) {
    return ACCEL_CMD;  
  }
  if ( cmd1 == 'd' && cmd2 == 'c' ) {
    return DECEL_CMD;  
  }
  if ( cmd1 == 'c' && cmd2 == 'd' ) {
    return CH_DIR_CMD;  
  }
  if ( cmd1 == 't' && cmd2 == 'd' ) {
    return DEST_CMD;  
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

short parse_sensor_name( char *cmd_buffer, int *buf_ind_ptr ) {
  short sensor_num = 0;
  char module_c = cmd_buffer[(*buf_ind_ptr)++];
  char first_num = cmd_buffer[(*buf_ind_ptr)++];
  char second_num = cmd_buffer[(*buf_ind_ptr)];
  sensor_num += ( module_c - 'A' ) * 16;
  if( second_num == ' ' || second_num == 0 ) {
    sensor_num += (first_num - '1');
  } else {
    if( first_num != '0' ) {
      sensor_num += (first_num - '0') * 10;
    }
    sensor_num += (second_num - '1');
    ++(*buf_ind_ptr);
  }
  return sensor_num;
}

int handle_move( char *cmd_buffer, short *train_speeds, rail_msg_t *rail_msg, int rail_server_tid ) {
  int buf_ind = 3;
  short train = parse_short( cmd_buffer, &buf_ind );
  ++buf_ind;
  short speed = parse_short( cmd_buffer, &buf_ind );

  if ( train > 0 && speed >= 0 ){
    ((rail_msg->to_server_content).rail_cmds)->train_id = train;
    if( speed == 0 ) {
      ((rail_msg->to_server_content).rail_cmds)->train_action = TR_STOP;
    } else {
      ((rail_msg->to_server_content).rail_cmds)->train_action = TR_CHANGE_SPEED;
    }
    ((rail_msg->to_server_content).rail_cmds)->train_speed = speed;
    ((rail_msg->to_server_content).rail_cmds)->train_delay = 0;

    Send( rail_server_tid, (char *)rail_msg, sizeof( *rail_msg ), (char *)&buf_ind, 0 );
    Printf( COM2, "\0337\033[1A\033[2K\rTrain %d set to %d\0338", train, speed );
  } else {
    output_invalid( );
    return -1;
  }
  return 0;
}

int handle_rev( char *cmd_buffer, short *train_speeds, rail_msg_t *rail_msg, int rail_server_tid ) {
  int buf_ind = 3;
  short train = parse_short( cmd_buffer, &buf_ind );
  //short prev_speed;
  if ( train <= 0 ){
    output_invalid( );
    return -1;
  }
  ((rail_msg->to_server_content).rail_cmds)->train_id = train;
  ((rail_msg->to_server_content).rail_cmds)->train_action = TR_REVERSE;
  ((rail_msg->to_server_content).rail_cmds)->train_speed = -1;
  ((rail_msg->to_server_content).rail_cmds)->train_delay = 0;
  Send( rail_server_tid, (char *)rail_msg, sizeof( *rail_msg ), (char *)&buf_ind, 0 );
  Printf( COM2, "\0337\033[1A\033[2K\rTrain %d reversed\0338", train );
  return 0;
}

int handle_init( char *cmd_buffer, rail_msg_t *rail_msg, int rail_server_tid ) {
  int buf_ind = 3;
  short train = parse_short( cmd_buffer, &buf_ind );
  //short prev_speed;
  if ( train <= 0 ){
    output_invalid( );
    return -1;
  }
  ((rail_msg->to_server_content).rail_cmds)->train_id = train;
  ((rail_msg->to_server_content).rail_cmds)->train_action = TR_INIT;
  ((rail_msg->to_server_content).rail_cmds)->train_speed = -1;
  ((rail_msg->to_server_content).rail_cmds)->train_delay = 0;
  Send( rail_server_tid, (char *)rail_msg, sizeof( *rail_msg ), (char *)&buf_ind, 0 );
  Printf( COM2, "\0337\033[1A\033[2K\rTrain %d initializing\0338", train );
  return 0;
}

// NEED TO SEND
int handle_switch( char *cmd_buffer, rail_msg_t *rail_msg, int rail_server_tid  ) {
  int buf_ind = 3;
  short switch_num = parse_short( cmd_buffer, &buf_ind );
  ++buf_ind;
  short c_s = parse_curve_straight( cmd_buffer, &buf_ind );
  char c_s_c;
  int state = -1;
  if ( switch_num <= 0 ||
       c_s <= 0 ||
       ( switch_num > 18 && !( switch_num >= 153 && switch_num <= 156 ) ) ) {
    output_invalid( );
    return -1;  
  }
  switch( c_s ) {
  case STRAIGHT:
    c_s_c = 'S';
    state = SW_STRAIGHT;
    break;
  case CURVED:
    c_s_c = 'C';
    state = SW_CURVED;
    break;
  default:
    c_s_c = '/';
    return -1;
    break;
  }

  pack_switch_cmd( rail_msg->to_server_content.rail_cmds, switch_num, state, 0 );
  Send( rail_server_tid, (char *)rail_msg, sizeof( *rail_msg ), (char *)&buf_ind, 0 );
  Printf( COM2, "\0337\033[1A\033[2K\rSwitch %d set to %c\0338", switch_num, c_s_c );
  return 0;
}

int handle_dest( char *cmd_buffer, rail_msg_t *rail_msg, int rail_server_tid  ) {
  int buf_ind = 3;
  short train = parse_short( cmd_buffer, &buf_ind );
  ++buf_ind;
  short dest = parse_sensor_name( cmd_buffer, &buf_ind );
  ++buf_ind;
  short mm_past_dest = parse_short( cmd_buffer, &buf_ind );
  ++buf_ind;
  char out_buf[3];

  if ( train <= 0 ){
    output_invalid( );
    return -1;
  }
  if( dest < -1 || dest >= 80 ) {
    output_invalid( );
    return -1;
  }

  ((rail_msg->to_server_content).rail_cmds)->train_id = train;
  ((rail_msg->to_server_content).rail_cmds)->train_action = TR_DEST;
  ((rail_msg->to_server_content).rail_cmds)->train_speed = -1;
  ((rail_msg->to_server_content).rail_cmds)->train_delay = 0;
  ((rail_msg->to_server_content).rail_cmds)->train_dest = dest;
  ((rail_msg->to_server_content).rail_cmds)->train_mm_past_dest = mm_past_dest;
  
  Send( rail_server_tid, (char *)rail_msg, sizeof( *rail_msg ), (char *)&buf_ind, 0 );
  sensor_id_to_name( dest, out_buf );
  if( dest == -1 ) {
    Printf( COM2, "\0337\033[1A\033[2K\rTrain %d is no longer destined for anything. =(\0338", train );
  } else {
    Printf( COM2, "\0337\033[1A\033[2K\rTrain %d is destined for %dmm past %c%c%c\0338", train, mm_past_dest, out_buf[0], out_buf[1], out_buf[2] );
  }
  return 0;
}

int handle_accel( char *cmd_buffer, rail_msg_t *rail_msg, int rail_server_tid  ) {
  int buf_ind = 3;
  short train = parse_short( cmd_buffer, &buf_ind );
  ++buf_ind;
  short accel_rate = parse_short( cmd_buffer, &buf_ind );
  ++buf_ind;

  if ( train <= 0 ){
    output_invalid( );
    return -1;
  }
  ((rail_msg->to_server_content).rail_cmds)->train_id = train;
  ((rail_msg->to_server_content).rail_cmds)->train_action = TR_ACCEL;
  ((rail_msg->to_server_content).rail_cmds)->train_accel = accel_rate;
  
  Send( rail_server_tid, (char *)rail_msg, sizeof( *rail_msg ), (char *)&buf_ind, 0 );
  Printf( COM2, "\0337\033[1A\033[2K\rTrain %d is acceleration rate set for %d.\0338", train, accel_rate );
  return 0;
}

int handle_decel( char *cmd_buffer, rail_msg_t *rail_msg, int rail_server_tid  ) {
  int buf_ind = 3;
  short train = parse_short( cmd_buffer, &buf_ind );
  ++buf_ind;
  short decel_rate = parse_short( cmd_buffer, &buf_ind );
  ++buf_ind;

  if ( train <= 0 ){
    output_invalid( );
    return -1;
  }
  ((rail_msg->to_server_content).rail_cmds)->train_id = train;
  ((rail_msg->to_server_content).rail_cmds)->train_action = TR_DECEL;
  ((rail_msg->to_server_content).rail_cmds)->train_decel = decel_rate;
  
  Send( rail_server_tid, (char *)rail_msg, sizeof( *rail_msg ), (char *)&buf_ind, 0 );
  Printf( COM2, "\0337\033[1A\033[2K\rTrain %d is deceleration rate set for %d.\0338", train, decel_rate );
  return 0;
}

int handle_ch_dir( char *cmd_buffer, rail_msg_t *rail_msg, int rail_server_tid  ) {
  int buf_ind = 3;
  short train = parse_short( cmd_buffer, &buf_ind );

  if ( train <= 0 ){
    output_invalid( );
    return -1;
  }
  ((rail_msg->to_server_content).rail_cmds)->train_id = train;
  ((rail_msg->to_server_content).rail_cmds)->train_action = TR_CH_DIR;
  
  Send( rail_server_tid, (char *)rail_msg, sizeof( *rail_msg ), (char *)&buf_ind, 0 );
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

int handle_quit( ) {
  return QUIT_CMD;
}

int handle_clear( ) {
  Printf( COM2, "\033[2J" );
  return 0;
}

int process_buffer( char *cmd_buffer, short *train_speeds, rail_msg_t *rail_msg, int rail_server_tid ) {
  int cmd = get_cmd( cmd_buffer );

  switch( cmd ) {
  case MOVE_CMD:
    handle_move( cmd_buffer, train_speeds, rail_msg, rail_server_tid );
    break;
  case REV_CMD:
    handle_rev( cmd_buffer, train_speeds, rail_msg, rail_server_tid );
    break;
  case SWITCH_CMD:
    handle_switch( cmd_buffer, rail_msg, rail_server_tid );
    break;
  case INIT_CMD:
    handle_init( cmd_buffer, rail_msg, rail_server_tid );
    break;
  case DEST_CMD:
    handle_dest( cmd_buffer, rail_msg, rail_server_tid );
    break;
  case ACCEL_CMD:
    handle_accel( cmd_buffer, rail_msg, rail_server_tid );
    break;
  case DECEL_CMD:
    handle_decel( cmd_buffer, rail_msg, rail_server_tid );
    break;
  case CH_DIR_CMD:
    handle_ch_dir( cmd_buffer, rail_msg, rail_server_tid );
    break;
  case GO_CMD:
    handle_go( );
    break;
  case KILL_CMD:
    handle_kill( );
    break;
  case CLEAR_CMD:
    handle_clear( );
    break;
  case QUIT_CMD:
    return handle_quit( );
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
  rail_cmds_t rail_cmds;
  init_rail_cmds( &rail_cmds );
  rail_msg_t rail_msg;
  rail_msg.request_type = USER_INPUT;
  rail_msg.to_server_content.rail_cmds = &rail_cmds;
  rail_msg.from_server_content.nullptr = NULL;
  int cmd_server_tid = WhoIs( (char*)CMD_SERVER );
  assertum( 1, cmd_server_tid >= 0, "cmd_server is not registered" );

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
      //Putstr( COM2, "\033[24;0H\033[2K\033[24;0H>", 19 );
      Printf( COM2, "$->" );
      init_rail_cmds( &rail_cmds );
      status = process_buffer( cmd_buffer, train_speeds, &rail_msg, cmd_server_tid );
      init_rail_cmds( &rail_cmds );
      //rail_cmds.train_id = 0;
      //rail_cmds.train_action = -1;
      //rail_cmds.train_delay = 0;
      rail_cmds.train_speed = 0;
      rail_cmds.train_dest = -1;
      rail_cmds.train_mm_past_dest = 0;
      rail_cmds.train_accel = 100;
      rail_cmds.train_accel = 120;
      //rail_cmds.switch_id0 = 0;
      //rail_cmds.switch_action0 = -1;
      //rail_cmds.switch_delay0 = 0;
      if( status == QUIT_CMD ) {
        //Putstr( COM2, "\0337\033[1A\033[2K\rShutting down. Goodbye!\033[24;0H\033[2K", 45 );
        Putstr( COM2, "Shutting down. Goodbye!\n\r", 45 );
        for( i = 12; i < NUM_TRAINS; ++i ) {
          train_speeds[i] = 0;
          set_train_speed_old( i, 0 );
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


