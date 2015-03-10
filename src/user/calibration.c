#include "calibration.h"
#include "io.h"
#include "track.h"
#include "clock_server.h"
#include "tools.h"
#include "syscall.h"
#include "rail_control.h"
#include "rail_helper.h"

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
  set_train_speed_old( TRAIN_NUM, 30 );
  for( i = 14; i > 7; --i ) {
    set_switch_old( 17, STRAIGHT );
    set_switch_old( 13, STRAIGHT );
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
    set_train_speed_old( TRAIN_NUM, i );
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
  set_train_speed_old( TRAIN_NUM, 0 );
  for( i = 8; i <= 14; ++i ) {
    set_switch_old( 17, STRAIGHT );
    set_switch_old( 13, STRAIGHT );
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
    set_train_speed_old( TRAIN_NUM, i );
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
  set_train_speed_old( TRAIN_NUM, 0 ); 
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
  set_switch_old( 17, STRAIGHT );
  set_switch_old( 13, STRAIGHT );
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
              set_train_speed_old( TRAIN_NUM, 0 );
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
  set_train_speed_old( TRAIN_NUM, 0 );
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
  train_state_t trains[TR_MAX];
  init_trains( trains, NULL, NULL );
  set_switch_old( 17, STRAIGHT );
  set_switch_old( 13, STRAIGHT );
  Putstr( COM1, &request_sensor, 1 );
  for( k = 0; k < NUM_SENSOR_BYTES; ++k ) {
    c = (char) Getc( COM1 );
  }
  set_train_speed_old( TRAIN_NUM, 14 );
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
                  set_train_speed_old( TRAIN_NUM, j ) ;
                  break;
                case 910: // E10
                  t2 = Time( ) - t0;
                  if( t0 ) {
                    set_train_speed_old( TRAIN_NUM, i );
                    v0 = trains[TRAIN_58].speeds[i].straight_vel;
                    v1 = trains[TRAIN_58].speeds[j].straight_vel;
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
  set_train_speed_old( TRAIN_NUM, 0 );
  Exit( );
}

/* TODO: Calibrate stopping time */
/* 
  Things to calibrate:
  - More accurate stopping distance - HIGH PRI - TONY - MON
    - get value for accel/decel
    - accel is more than decel
  - Update train position and velocity constantly (in idle task?)
  - stopping time (once we have velocity and stopping distance) - Can do mathematically, = (2*d)/v0 - HIGH PRI - TONY - MON
  - update train speed on set_train_speed_old // HIGH PRI - TONY - MON
  - update_costs( graph, velocity ) // MED-PRI - TONY
  - Calculate reverse costs - SUN (LOW PRI - TONY)
  - Assert ring buffer - LOW PRI - TONY
  - Calibrate another train - VERY LOW PRI

  - train initialization. how? - HIGH PRI - TONY/WILSON

  - Next sensor prediction (time it takes to hit next sensor) - SAT (HIGH PRI, WILSON)
    - What's the next sensor?
    - get_next_sensor and distance static DONE, need more testing
    - get_next_sensor and distance dynamic
  - sensor hit to train mapping - WILSON

  - How long till next sensor? // DONE, need testing
  - time_to_next_sensor // DONE, need testing
    - NOTE: Update this after graph search / speed changes
*/

