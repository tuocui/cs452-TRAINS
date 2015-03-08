#include "calibration.h"
#include "io.h"
#include "track.h"
#include "clock_server.h"
#include "tools.h"
#include "syscall.h"
#include "rail_control.h"

#define TRAIN_NUM 58
#define WEIGHT_PREV 50
#define WEIGHT_NEW 50

void calibrate_train_velocity( ) {
  //int train_num = TRAIN_NUM;
  char c;
  int module_num = 0;
  int sensor_num = 1;
  int recent_sensor;
  int most_recent_sensor = 0;
  int recent_sensors[NUM_RECENT_SENSORS];
  int recent_sensors_ind = 0;
  int num_sensors_triggered = 0;
  int recent_sensor_triggered = 0;
  char request_sensor = REQUEST_SENSOR;
  int i = 14;
  int j = 0;
  int k = 0;
  int l = 0;
  int cur_time;
  set_train_speed( TRAIN_NUM, 30 );
  for( i = 14; i > 7; --i ) {
    set_switch( 17, STRAIGHT );
    set_switch( 13, STRAIGHT );
    int str_nsw_l = 0;
    int str_nsw_t = 0;
    int str_nsw_num = 0;
    int str_nsw_avg = 0;
    int str_sw_avg = 0;
    int tight_nsw_avg = 0;
    int tight_sw_avg = 0;
    int str_sw_l = 0;
    int str_sw_t = 0;
    int str_sw_num = 0;
    int tight_sw_l = 0;
    int tight_sw_t = 0;
    int tight_sw_num = 0;
    int tight_nsw_l = 0;
    int tight_nsw_t = 0;
    int tight_nsw_num = 0;
    /*int loose_sw_l = 0;
    int loose_sw_t = 0;
    int loose_sw_num = 0;
    int loose_sw_avg = 0;
    int loose_nsw_l = 0;
    int loose_nsw_t = 0;
    int loose_nsw_num = 0;
    int loose_nsw_avg = 0;*/
    int d3_rdy = 0;
    int e13_rdy = 0;
    int b5_rdy = 0;
    int d13_rdy = 0;
    int c9_rdy = 0;
    int a3_rdy = 0;
    int e5_rdy = 0;
    int e10_rdy = 0;
    //int c6_rdy = 0;
    //int e7_rdy = 0;
    //int d9_rdy = 0;
    set_train_speed( TRAIN_NUM, i );
    Delay( 1500 );
    Putstr( COM1, &request_sensor, 1 );
    for( k = 0; k < NUM_SENSOR_BYTES; ++k ) {
      c = (char) Getc( COM1 );
    }
    for( j = 0; j < 10 ; ) {
      Putstr( COM1, &request_sensor, 1 );
      module_num = 0;
      recent_sensor_triggered = 0;
      for( k = 0; k < NUM_SENSOR_BYTES; ++k ) {
        c = (char) Getc( COM1 );
        sensor_num = 1;
        if ( c > 0 ) {
          if ( module_num % 2 == 1 ) {
            sensor_num += 8;
          }
          for( l = 0; l < 8 ; ++l ) {
            // Yay for bitwise operations
            if ( ( c >> ( 7 - l ) ) & 0x1 ) {
              recent_sensor = ( module_num * 100 ) + sensor_num;
              if ( recent_sensor == most_recent_sensor ) {
                ++sensor_num;
                continue;
              }
              recent_sensor_triggered = 1;
              recent_sensors[recent_sensors_ind] = recent_sensor;
              ++num_sensors_triggered;
              most_recent_sensor = recent_sensor;
              recent_sensors_ind = ( recent_sensors_ind + 1 ) % NUM_RECENT_SENSORS;
              //Printf( COM2, "sensor: %d\r\n", recent_sensor );
              switch( recent_sensor ) {
              case 603: // D3
                ++j;
                d3_rdy = Time( );
                if( b5_rdy ) {
                  str_nsw_l = ( 404 * 10000 ) / ( d3_rdy - b5_rdy );
                  if( str_nsw_avg == 0 ) {
                    str_nsw_avg = str_nsw_l;
                  }
                  str_nsw_avg = ( ( WEIGHT_PREV * str_nsw_avg ) + ( WEIGHT_NEW * str_nsw_l ) ) / 100;
                  //str_nsw_avg = (str_nsw_t * 100) / str_nsw_num;
                  b5_rdy = 0;
                }
                if( j > 2 ) {
                  if( str_nsw_avg > 0 ) {
                    str_nsw_t += str_nsw_avg;
                    ++str_nsw_num;
                  }
                  if( str_sw_avg > 0 ) {
                    str_sw_t += str_sw_avg;
                    ++str_sw_num;
                  }
                  if( tight_nsw_avg > 0 ) {
                    tight_nsw_t += tight_nsw_avg;
                    ++tight_nsw_num;
                  }
                  if( tight_sw_avg > 0 ) {
                    tight_sw_t += tight_sw_avg;
                    ++tight_sw_num;
                  }
                }
                //Printf( COM2, "Speed: %d LOW - str_sw_avg: %d, str_nsw_avg: %d, t_sw_avg: %d, t_nsw_avg: %d\r\n", i, str_sw_avg, str_nsw_avg, tight_sw_avg, tight_nsw_avg );
                break;
              case 805: // E5
                e5_rdy = Time( );
                if( d3_rdy ) {
                  ++str_sw_num;
                  str_sw_l = ( 289 * 10000 ) / ( e5_rdy - d3_rdy );
                  str_sw_t += str_sw_l;
                  if( str_sw_avg == 0 ) {
                    str_sw_avg = str_sw_l;
                  }
                  str_sw_avg = ( ( WEIGHT_PREV * str_sw_avg ) + ( WEIGHT_NEW * str_sw_l ) ) / 100;
                  //str_sw_avg = (str_sw_t * 100) / str_sw_num;
                  d3_rdy = 0;
                }
                break;
              case 913: // E13
                e13_rdy = Time( );
                if( e10_rdy ) {
                  ++tight_nsw_num;
                  tight_nsw_l = ( 282 * 10000 ) / ( e13_rdy - e10_rdy );
                  tight_nsw_t += tight_nsw_l;
                  if( tight_nsw_avg == 0 ) {
                    tight_nsw_avg = tight_nsw_l;
                  }
                  tight_nsw_avg = ( ( WEIGHT_PREV * tight_nsw_avg ) + ( WEIGHT_NEW * tight_nsw_l ) ) / 100;
                  e10_rdy = 0;
                }
                break;
              case 713: // D13
                d13_rdy = Time( );
                if( e13_rdy ) {
                  ++str_sw_num;
                  str_sw_l = ( 282 * 10000 ) / ( d13_rdy - e13_rdy );
                  str_sw_t += str_sw_l;
                  if( str_sw_avg == 0 ) {
                    str_sw_avg = str_sw_l;
                  }
                  str_sw_avg = ( ( WEIGHT_PREV * str_sw_avg ) + ( WEIGHT_NEW * str_sw_l ) ) / 100;
                  //str_sw_avg = (str_sw_t * 100) / str_sw_num;
                  e13_rdy = 0;
                }
                break;
              case 205: // B5
                b5_rdy = Time( );
                break;
              case 202: // B2
                cur_time = Time( );
                if( d13_rdy ) {
                  ++str_nsw_num;
                  str_nsw_l = ( 404 * 10000 ) / ( cur_time - d13_rdy );
                  str_nsw_t += str_nsw_l;
                  if( str_nsw_avg == 0 ) {
                    str_nsw_avg = str_nsw_l;
                  }
                  str_nsw_avg = ( ( WEIGHT_PREV * str_nsw_avg ) + ( WEIGHT_NEW * str_nsw_l ) ) / 100;
                  //str_nsw_avg = (str_nsw_t * 100) / str_nsw_num;
                  d13_rdy = 0;
                }
                break;
              case 509: // C9
                c9_rdy = Time( );
                break;
              case 315: //B15
                cur_time = Time( );
                if( c9_rdy ) {
                  ++tight_sw_num;
                  tight_sw_l = ( 376 * 10000 ) / ( cur_time - c9_rdy );
                  tight_sw_t += tight_sw_l;
                  if( tight_sw_avg == 0 ) {
                    tight_sw_avg = tight_sw_l;
                  }
                  tight_sw_avg = ( ( WEIGHT_PREV * tight_sw_avg ) + ( WEIGHT_NEW * tight_sw_l ) ) / 100;
                  //tight_sw_avg = (tight_sw_t * 100) / tight_sw_num;
                  c9_rdy = 0;
                }
                break;
              case 3: // A3
                a3_rdy = Time( );
                break;
              case 511: // C11
                cur_time = Time( );
                if( a3_rdy ) {
                  ++tight_sw_num;
                  tight_sw_l = ( 376 * 10000 ) / ( cur_time - a3_rdy );
                  tight_sw_t += tight_sw_l;
                  if( tight_sw_avg == 0 ) {
                    tight_sw_avg = tight_sw_l;
                  }
                  tight_sw_avg = ( ( WEIGHT_PREV * tight_sw_avg ) + ( WEIGHT_NEW * tight_sw_l ) ) / 100;
                  //tight_sw_avg = (tight_sw_t * 100) / tight_sw_num;
                  a3_rdy = 0;
                }
                break;
              case 605: // D5
                cur_time = Time( );
                if( e5_rdy ) {
                  ++tight_nsw_num;
                  tight_nsw_l = ( 282 * 10000 ) / ( cur_time - e5_rdy );
                  tight_nsw_t += tight_nsw_l;
                  if( tight_nsw_avg == 0 ) {
                    tight_nsw_avg = tight_nsw_l;
                  }
                  tight_nsw_avg = ( ( WEIGHT_PREV * tight_nsw_avg ) + ( WEIGHT_NEW * tight_nsw_l ) ) / 100;
                  //tight_nsw_avg = (tight_nsw_t * 100) / tight_nsw_num;
                  e5_rdy = 0;
                }
                break;
              case 910: // E10
                e10_rdy = Time( );
                break;
              default:
                break;
              }
            }
            ++sensor_num;
          }
        }
        ++module_num;
      }
    }
    Printf( COM2, "trains[%d].speeds[%d].straight_vel = %d;\r\n", TRAIN_NUM, i, str_nsw_t / str_nsw_num );
    Printf( COM2, "trains[%d].speeds[%d].curved_vel = %d;\r\n", TRAIN_NUM, i, tight_nsw_t / tight_nsw_num );
    //Printf( COM2, "Going over a switch loses: %dms\r\n", ( ( str_nsw_t / str_nsw_num ) * 285 ) - ( str_sw_t / str_sw_num ) * 285 );
  }
  set_train_speed( TRAIN_NUM, 0 );
  for( i = 8; i <= 14; ++i ) {
    set_switch( 17, STRAIGHT );
    set_switch( 13, STRAIGHT );
    int str_nsw_l = 0;
    int str_nsw_t = 0;
    int str_nsw_num = 0;
    int str_nsw_avg = 0;
    int str_sw_avg = 0;
    int tight_nsw_avg = 0;
    int tight_sw_avg = 0;
    int str_sw_l = 0;
    int str_sw_t = 0;
    int str_sw_num = 0;
    int tight_sw_l = 0;
    int tight_sw_t = 0;
    int tight_sw_num = 0;
    int tight_nsw_l = 0;
    int tight_nsw_t = 0;
    int tight_nsw_num = 0;
    /*int loose_sw_l = 0;
    int loose_sw_t = 0;
    int loose_sw_num = 0;
    int loose_sw_avg = 0;
    int loose_nsw_l = 0;
    int loose_nsw_t = 0;
    int loose_nsw_num = 0;
    int loose_nsw_avg = 0;*/
    int d3_rdy = 0;
    int e13_rdy = 0;
    int b5_rdy = 0;
    int d13_rdy = 0;
    int c9_rdy = 0;
    int a3_rdy = 0;
    int e5_rdy = 0;
    int e10_rdy = 0;
    //int c6_rdy = 0;
    //int e7_rdy = 0;
    //int d9_rdy = 0;
    set_train_speed( TRAIN_NUM, i );
    Delay( 1500 );
    Putstr( COM1, &request_sensor, 1 );
    for( k = 0; k < NUM_SENSOR_BYTES; ++k ) {
      c = (char) Getc( COM1 );
    }
    for( j = 0; j < 10 ; ) {
      Putstr( COM1, &request_sensor, 1 );
      module_num = 0;
      recent_sensor_triggered = 0;
      for( k = 0; k < NUM_SENSOR_BYTES; ++k ) {
        c = (char) Getc( COM1 );
        sensor_num = 1;
        if ( c > 0 ) {
          if ( module_num % 2 == 1 ) {
            sensor_num += 8;
          }
          for( l = 0; l < 8 ; ++l ) {
            // Yay for bitwise operations
            if ( ( c >> ( 7 - l ) ) & 0x1 ) {
              recent_sensor = ( module_num * 100 ) + sensor_num;
              if ( recent_sensor == most_recent_sensor ) {
                ++sensor_num;
                continue;
              }
              recent_sensor_triggered = 1;
              recent_sensors[recent_sensors_ind] = recent_sensor;
              ++num_sensors_triggered;
              most_recent_sensor = recent_sensor;
              recent_sensors_ind = ( recent_sensors_ind + 1 ) % NUM_RECENT_SENSORS;
              //Printf( COM2, "sensor: %d\r\n", recent_sensor );
              switch( recent_sensor ) {
              case 603: // D3
                ++j;
                d3_rdy = Time( );
                if( b5_rdy ) {
                  str_nsw_l = ( 404 * 10000 ) / ( d3_rdy - b5_rdy );
                  if( str_nsw_avg == 0 ) {
                    str_nsw_avg = str_nsw_l;
                  }
                  str_nsw_avg = ( ( WEIGHT_PREV * str_nsw_avg ) + ( WEIGHT_NEW * str_nsw_l ) ) / 100;
                  //str_nsw_avg = (str_nsw_t * 100) / str_nsw_num;
                  b5_rdy = 0;
                }
                if( j > 2 ) {
                  if( str_nsw_avg > 0 ) {
                    str_nsw_t += str_nsw_avg;
                    ++str_nsw_num;
                  }
                  if( str_sw_avg > 0 ) {
                    str_sw_t += str_sw_avg;
                    ++str_sw_num;
                  }
                  if( tight_nsw_avg > 0 ) {
                    tight_nsw_t += tight_nsw_avg;
                    ++tight_nsw_num;
                  }
                  if( tight_sw_avg > 0 ) {
                    tight_sw_t += tight_sw_avg;
                    ++tight_sw_num;
                  }
                }
                //Printf( COM2, "Speed: %d HIGH - str_sw_avg: %d, str_nsw_avg: %d, t_sw_avg: %d, t_nsw_avg: %d\r\n", i, str_sw_avg, str_nsw_avg, tight_sw_avg, tight_nsw_avg );
                break;
              case 805: // E5
                e5_rdy = Time( );
                if( d3_rdy ) {
                  ++str_sw_num;
                  str_sw_l = ( 289 * 10000 ) / ( e5_rdy - d3_rdy );
                  str_sw_t += str_sw_l;
                  if( str_sw_avg == 0 ) {
                    str_sw_avg = str_sw_l;
                  }
                  str_sw_avg = ( ( WEIGHT_PREV * str_sw_avg ) + ( WEIGHT_NEW * str_sw_l ) ) / 100;
                  //str_sw_avg = (str_sw_t * 100) / str_sw_num;
                  d3_rdy = 0;
                }
                break;
              case 913: // E13
                e13_rdy = Time( );
                if( e10_rdy ) {
                  ++tight_nsw_num;
                  tight_nsw_l = ( 282 * 10000 ) / ( e13_rdy - e10_rdy );
                  tight_nsw_t += tight_nsw_l;
                  if( tight_nsw_avg == 0 ) {
                    tight_nsw_avg = tight_nsw_l;
                  }
                  tight_nsw_avg = ( ( WEIGHT_PREV * tight_nsw_avg ) + ( WEIGHT_NEW * tight_nsw_l ) ) / 100;
                  e10_rdy = 0;
                }
                break;
              case 713: // D13
                d13_rdy = Time( );
                if( e13_rdy ) {
                  ++str_sw_num;
                  str_sw_l = ( 282 * 10000 ) / ( d13_rdy - e13_rdy );
                  str_sw_t += str_sw_l;
                  if( str_sw_avg == 0 ) {
                    str_sw_avg = str_sw_l;
                  }
                  str_sw_avg = ( ( WEIGHT_PREV * str_sw_avg ) + ( WEIGHT_NEW * str_sw_l ) ) / 100;
                  //str_sw_avg = (str_sw_t * 100) / str_sw_num;
                  e13_rdy = 0;
                }
                break;
              case 205: // B5
                b5_rdy = Time( );
                break;
              case 202: // B2
                cur_time = Time( );
                if( d13_rdy ) {
                  ++str_nsw_num;
                  str_nsw_l = ( 404 * 10000 ) / ( cur_time - d13_rdy );
                  str_nsw_t += str_nsw_l;
                  if( str_nsw_avg == 0 ) {
                    str_nsw_avg = str_nsw_l;
                  }
                  str_nsw_avg = ( ( WEIGHT_PREV * str_nsw_avg ) + ( WEIGHT_NEW * str_nsw_l ) ) / 100;
                  //str_nsw_avg = (str_nsw_t * 100) / str_nsw_num;
                  d13_rdy = 0;
                }
                break;
              case 509: // C9
                c9_rdy = Time( );
                break;
              case 315: //B15
                cur_time = Time( );
                if( c9_rdy ) {
                  ++tight_sw_num;
                  tight_sw_l = ( 376 * 10000 ) / ( cur_time - c9_rdy );
                  tight_sw_t += tight_sw_l;
                  if( tight_sw_avg == 0 ) {
                    tight_sw_avg = tight_sw_l;
                  }
                  tight_sw_avg = ( ( WEIGHT_PREV * tight_sw_avg ) + ( WEIGHT_NEW * tight_sw_l ) ) / 100;
                  //tight_sw_avg = (tight_sw_t * 100) / tight_sw_num;
                  c9_rdy = 0;
                }
                break;
              case 3: // A3
                a3_rdy = Time( );
                break;
              case 511: // C11
                cur_time = Time( );
                if( a3_rdy ) {
                  ++tight_sw_num;
                  tight_sw_l = ( 376 * 10000 ) / ( cur_time - a3_rdy );
                  tight_sw_t += tight_sw_l;
                  if( tight_sw_avg == 0 ) {
                    tight_sw_avg = tight_sw_l;
                  }
                  tight_sw_avg = ( ( WEIGHT_PREV * tight_sw_avg ) + ( WEIGHT_NEW * tight_sw_l ) ) / 100;
                  //tight_sw_avg = (tight_sw_t * 100) / tight_sw_num;
                  a3_rdy = 0;
                }
                break;
              case 605: // D5
                cur_time = Time( );
                if( e5_rdy ) {
                  ++tight_nsw_num;
                  tight_nsw_l = ( 282 * 10000 ) / ( cur_time - e5_rdy );
                  tight_nsw_t += tight_nsw_l;
                  if( tight_nsw_avg == 0 ) {
                    tight_nsw_avg = tight_nsw_l;
                  }
                  tight_nsw_avg = ( ( WEIGHT_PREV * tight_nsw_avg ) + ( WEIGHT_NEW * tight_nsw_l ) ) / 100;
                  //tight_nsw_avg = (tight_nsw_t * 100) / tight_nsw_num;
                  e5_rdy = 0;
                }
                break;
              case 910: // E10
                e10_rdy = Time( );
                break;
              default:
                break;
              }
            }
            ++sensor_num;
          }
        }
        ++module_num;
      }
    }
    Printf( COM2, "trains[%d].speeds[%d].straight_vel = %d;\r\n", TRAIN_NUM, i + 15, str_nsw_t / str_nsw_num );
    Printf( COM2, "trains[%d].speeds[%d].curved_vel = %d;\r\n", TRAIN_NUM, i + 15, tight_nsw_t / tight_nsw_num );
    //Printf( COM2, "Going over a switch loses: %d\r\n", ( ( str_nsw_t / str_nsw_num ) * 285 ) - ( str_sw_t / str_sw_num ) * 285 );
  }
  set_train_speed( TRAIN_NUM, 0 ); 
  Exit( );
}

void calibrate_stopping_distance( ) {
  //int train_num = TRAIN_NUM;
  char c;
  int module_num = 0;
  int sensor_num = 1;
  int recent_sensor;
  int most_recent_sensor = 0;
  int recent_sensors[NUM_RECENT_SENSORS];
  int recent_sensors_ind = 0;
  int num_sensors_triggered = 0;
  int recent_sensor_triggered = 0;
  char request_sensor = REQUEST_SENSOR;
  int k = 0;
  int l = 0;
  set_switch( 17, STRAIGHT );
  set_switch( 13, STRAIGHT );
  Putstr( COM1, &request_sensor, 1 );
  for( k = 0; k < NUM_SENSOR_BYTES; ++k ) {
    c = (char) Getc( COM1 );
  }
  Delay( 500 ) ;
  FOREVER {
    Putstr( COM1, &request_sensor, 1 );
    module_num = 0;
    recent_sensor_triggered = 0;
    for( k = 0; k < NUM_SENSOR_BYTES; ++k ) {
      c = (char) Getc( COM1 );
      sensor_num = 1;
      if ( c > 0 ) {
        if ( module_num % 2 == 1 ) {
          sensor_num += 8;
        }
        for( l = 0; l < 8 ; ++l ) {
          // Yay for bitwise operations
          if ( ( c >> ( 7 - l ) ) & 0x1 ) {
            recent_sensor = ( module_num * 100 ) + sensor_num;
            if ( recent_sensor == most_recent_sensor ) {
              ++sensor_num;
              continue;
            }
            recent_sensor_triggered = 1;
            recent_sensors[recent_sensors_ind] = recent_sensor;
            ++num_sensors_triggered;
            most_recent_sensor = recent_sensor;
            recent_sensors_ind = ( recent_sensors_ind + 1 ) % NUM_RECENT_SENSORS;
            //Printf( COM2, "sensor: %d\r\n", recent_sensor );
            switch( recent_sensor ) {
            case 205: // B5
              set_train_speed( TRAIN_NUM, 0 );
              break;
            default:
              break;
            }
          }
          ++sensor_num;
        }
      }
      ++module_num;
    }
  }
  set_train_speed( TRAIN_NUM, 0 );
  Exit( );
}

// B5 -> E10
void calibrate_accel_time( ) {
  //int train_num = TRAIN_NUM;
  char c;
  int module_num = 0;
  int sensor_num = 1;
  int recent_sensor;
  int most_recent_sensor = 0;
  int recent_sensors[NUM_RECENT_SENSORS];
  int recent_sensors_ind = 0;
  int num_sensors_triggered = 0;
  int recent_sensor_triggered = 0;
  char request_sensor = REQUEST_SENSOR;
  int k = 0;
  int l = 0;
  int i = 0;
  int m;
  int j = 0;
  int t0 = 0;
  int t2;
  int dt = 1598;
  int t1;
  int v0;
  int v1;
  train_state_t trains[65];
  init_trains( trains, 65 );
  set_switch( 17, STRAIGHT );
  set_switch( 13, STRAIGHT );
  Putstr( COM1, &request_sensor, 1 );
  for( k = 0; k < NUM_SENSOR_BYTES; ++k ) {
    c = (char) Getc( COM1 );
  }
  set_train_speed( TRAIN_NUM, 14 );
  Delay( 500 ) ;
  recent_sensor_triggered = 0;
  for( i = 14; i > 7; --i ) {
    for( j = i - 1; j > 7; --j ) {
      for ( m = 0; m < 5; ) {
        Putstr( COM1, &request_sensor, 1 );
        module_num = 0;
        for( k = 0; k < NUM_SENSOR_BYTES; ++k ) {
          c = (char) Getc( COM1 );
          sensor_num = 1;
          if ( c > 0 ) {
            if ( module_num % 2 == 1 ) {
              sensor_num += 8;
            }
            for( l = 0; l < 8 ; ++l ) {
              // Yay for bitwise operations
              if ( ( c >> ( 7 - l ) ) & 0x1 ) {
                recent_sensor = ( module_num * 100 ) + sensor_num;
                if ( recent_sensor == most_recent_sensor ) {
                  ++sensor_num;
                  continue;
                }
                recent_sensor_triggered = 1;
                recent_sensors[recent_sensors_ind] = recent_sensor;
                ++num_sensors_triggered;
                most_recent_sensor = recent_sensor;
                recent_sensors_ind = ( recent_sensors_ind + 1 ) % NUM_RECENT_SENSORS;
                //Printf( COM2, "sensor: %d\r\n", recent_sensor );
                switch( recent_sensor ) {
                case 205: // B5
                  t0 = Time( );
                  set_train_speed( TRAIN_NUM, j ) ;
                  break;
                case 910: // E10
                  t2 = Time( ) - t0;
                  if( t0 ) {
                    set_train_speed( TRAIN_NUM, i );
                    v0 = trains[TRAIN_NUM].speeds[i].straight_vel;
                    v1 = trains[TRAIN_NUM].speeds[j].straight_vel;
                    t1 = (( (2*dt) - ( (2*t2*v1) / 10000 ) ) * 10000) / (v0 - v1);

                    Printf( COM2, "%d\r\n", t2);
                    Printf( COM2, "%d\r\n", (2*t2*v1) / 100000);
                    Printf( COM2, "%d\r\n", (2*t2*v1) / 10000);
                    Printf( COM2, "%d\r\n", (2*t2*v1) / 1000);
                    Printf( COM2, "%d\r\n", (2*t2*v1) / 100);
                    Printf( COM2, "%d\r\n", (2*t2*v1) / 10);
                    Printf( COM2, "Train: %d, speed: %d to %d, t1: %d\r\n", TRAIN_NUM, i, j, t1);
                    ++m;
                  }
                  v0 = 0;
                  v1 = 0;
                  t0 = 0;
                  t2 = 0;
                  t1 = 0;
                  break;
                default:
                  break;
                }
              }
              ++sensor_num;
            }
          }
          ++module_num;
        }
      }
    }
  }
  set_train_speed( TRAIN_NUM, 0 );
  Exit( );
}

/* TODO: Calibrate stopping time */
/* 
  Things to calibrate:
  - velocities (just need to hog track for a long ass time) DONE
  - stopping distance (also, need to hog track for a long ass time) DONE
  - stopping time (once we have velocity and stopping distance) - Can do mathematically, = (2*d)/v0
  - Figure out that accel/decel model, is it better to just model accel/decel as linear? - SAT
    - How what to assume for accel/decel distance/time? Can we assume a linear function with the difference in speed as x?
  - Calibrate velocity on the fly - SAT
  - How to send sensor signals to the controller?
  - Calculate reverse costs - SUN
  - Need to redo how sensor num calculated - SAT
  - Next sensor prediction (time it takes to hit next sensor) - SAT
    - What's the next sensor?
    - How long till next sensor?
    - Actual time hit next sensor?
*/

void init_trains( train_state_t *trains, int num_trains ) {
  int i;
  int j;
  for( i = 0; i < num_trains; ++i ) {
    trains[i].prev_node_id= 0;
    trains[i].next_node_id= 0;
    trains[i].nm_past_landmark = 0;
    trains[i].cur_speed = 0;
    for( j = 0; j < NUM_SPEEDS; ++j ) {
      trains[i].speeds[j].speed = 0;
      trains[i].speeds[j].high_low = 0;
      trains[i].speeds[j].straight_vel = 0;
      trains[i].speeds[j].curved_vel = 0;
      trains[i].speeds[j].stopping_distance = 0;
      trains[i].speeds[j].stopping_time = 0;
      trains[i].speeds[j].accel_distance = 0;
      trains[i].speeds[j].accel_time = 0;
    }
  }

  /* Copypasta calibration output here */
  // Velocity in mm/100s, divide by 1000 to get cm/s
  // Stopping distance in mm
  trains[58].speeds[14].straight_vel = 54179; // 14 LOW
  trains[58].speeds[14].curved_vel = 53686;
  trains[58].speeds[14].stopping_distance = 1188;
  trains[58].speeds[13].straight_vel = 53192; // 13 HIGH
  trains[58].speeds[13].curved_vel = 53983;
  trains[58].speeds[13].stopping_distance = 1052;
  trains[58].speeds[12].straight_vel = 48798; // 12 HIGH
  trains[58].speeds[12].curved_vel = 50517;
  trains[58].speeds[12].stopping_distance = 852;
  trains[58].speeds[11].straight_vel = 41440; // 11 HIGH
  trains[58].speeds[11].curved_vel = 41749;
  trains[58].speeds[11].stopping_distance = 645;
  trains[58].speeds[10].straight_vel = 33814; // 10 HIGH
  trains[58].speeds[10].curved_vel = 34627;
  trains[58].speeds[10].stopping_distance = 460;
  trains[58].speeds[9].straight_vel = 28004; // 9 HIGH
  trains[58].speeds[9].curved_vel = 27860;
  trains[58].speeds[9].stopping_distance = 336;
  trains[58].speeds[8].straight_vel = 22133; // 8 HIGH
  trains[58].speeds[8].curved_vel = 22370;
  trains[58].speeds[8].stopping_distance = 231;
  trains[58].speeds[23].straight_vel = 18907; // 8 LOW
  trains[58].speeds[23].curved_vel = 19127;
  trains[58].speeds[23].stopping_distance = 186;
  trains[58].speeds[24].straight_vel = 25265; // 9 LOW
  trains[58].speeds[24].curved_vel = 25270;
  trains[58].speeds[24].stopping_distance = 289;
  trains[58].speeds[25].straight_vel = 31219; // 10 LOW
  trains[58].speeds[25].curved_vel = 31030;
  trains[58].speeds[25].stopping_distance = 402;
  trains[58].speeds[26].straight_vel = 38121; // 11 LOW
  trains[58].speeds[26].curved_vel = 38043;
  trains[58].speeds[26].stopping_distance = 550;
  trains[58].speeds[27].straight_vel = 45551; // 12 LOW
  trains[58].speeds[27].curved_vel = 44540;
  trains[58].speeds[27].stopping_distance = 754;
  trains[58].speeds[28].straight_vel = 52030; // 13 LOW
  trains[58].speeds[28].curved_vel = 51056;
  trains[58].speeds[28].stopping_distance = 916;
  trains[58].speeds[29].straight_vel = 54344; // 14 LOW
  trains[58].speeds[29].curved_vel = 52635;
  trains[58].speeds[29].stopping_distance = 1188;
}

