#include "global.h"
#include "tools.h"

#include "syscall.h"
#include "nameserver.h"
#include "rail_server.h"
#include "rail_control.h"
#include "calibration.h"
#include "track.h"


/* a notifier, will change to couriers if necessary */
//NOTE: each train should have its own graph_search thread, becuase cmds is local
// and is not synchronized
void rail_graph_notifier( ) {
  int rail_server_tid = MyParentTid( );
  assert( 1, rail_server_tid > 0 );

  /* content to send to the server, first msg has NULL cmds */
  rail_cmds_t rail_cmds;
  init_rail_cmds( &rail_cmds ); 

  /* content to receive from the server */
  train_state_t train_state; 

  rail_msg_t rail_msg;  
  rail_msg.request_type = RAIL_CMDS;
  rail_msg.to_server_content.rail_cmds = &rail_cmds;
  rail_msg.from_server_content.train_state = &train_state;

  FOREVER {
    Send( rail_server_tid, (char*)&rail_msg, sizeof( rail_msg ), 
                           (char*)&rail_msg, sizeof( rail_msg ));
    assert( 1, rail_msg.from_server_content.train_state );

    get_next_command( &train_state, &rail_cmds );
    //TODO: anything else?
  }
}

void sensor_data_notifier( ) {
  int rail_server_tid = MyParentTid( );
  assert( 1, rail_server_tid > 0 );
  
  /* content to send to the server */
  sensor_data_t sensor_data;
  //TODO: should zero out for the first msg to avoid data corruption?
  
  rail_msg_t rail_msg;
  rail_msg.request_type = SENSOR_DATA;
  rail_msg.to_server_content.sensor_data = &sensor_data;
  rail_msg.from_server_content.nullptr = NULL; // as lean as possible

}

void rail_server( ) {
  if( RegisterAs( (char*)RAIL_SERVER ) == -1 ) {
    bwputstr( COM2, "ERROR: failed to register rail_server, aborting ...\n\r" );
    Exit( );
  }
  int client_tid;
  
  //int switch_states[SW_MAX];
  int train_graph_search_tid[TR_MAX]; 
  //bool train_graph_search_ready[TR_MAX];
  //bool train_delay_treads_arrived[TR_MAX]; TODO
  //train_state_t trains[TR_MAX];
  rail_msg_t receive_msg;

  int i;
  for( i = 0; i < TR_MAX; ++i ) {
    train_graph_search_tid[i] = Create( 10, &rail_graph_notifier);
    assert( 1, train_graph_search_tid[i] > 0 );
  }
  
  //TODO: add secretary/courier 
  FOREVER { 
    Receive( &client_tid, (char *)&receive_msg, sizeof( rail_msg_t ));

    switch( receive_msg.request_type ) {
      case SENSOR_DATA:
        /* retrieve data, find sensor number and corresponding train, set ready */
        break;
      case RAIL_CMDS:
        break;
      case CALIBRATION:
        break;
      case TRAIN_EXE_READY:
        break;
      case SWITCH_EXE_READY:
        break;
      default:
        assertm( 1, false, "ERROR: unrecognized request: %d", receive_msg.request_type );
      break;
    }
  }
}
