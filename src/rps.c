#include <tools.h>
#include <rps.h>
#include <nameserver.h>
#include <timer.h>
#include <syscall.h>

void initialize_clients( client_t *clients ) {
  int i = 0;
  for( ; i < TD_MAX - 1; ++i ) {
    clients[i].c_tid = 0;
    clients[i].next_client_q = NULL;
    clients[i].next_client_free = &(clients[i+1]);
  }
  clients[i].c_tid = 0;
  clients[i].next_client_q = NULL;
  clients[i].next_client_free = NULL;
}

void rps_server( ) {
  int errno;
  errno = RegisterAs( (char *) JOB_RPS );
  debug( "Woo, registered" );
  if( errno < 0 ) {
    bwputstr( COM2, "Error registering RPS server, aborting." );
    Exit( );
  }

  client_t clients[TD_MAX];
  initialize_clients( clients );
  client_t *first_client_q = NULL;
  client_t *last_client_q = NULL;
  client_t *new_client = NULL;
  client_t *first_client_free = &(clients[0]);
  client_t *last_client_free = &(clients[TD_MAX - 1]);
  
  rps_msg_t receive;
  rps_msg_t reply;
  unsigned int client_tid;
  int msg_size = sizeof( receive );
  //int num_clients = 0;

  client_t *player1 = NULL;
  client_t *player2 = NULL;
  int p1_action = -1;
  int p2_action = -1;
  short should_quit = 0;
  short p1_quit = 0;
  short p2_quit = 0;

  // used for "random" generator
  start_clock( TIMER_LOAD_VAL );

  FOREVER {
    Receive( (int *)&client_tid, (char *)&receive, msg_size );
    switch( receive.type ) {
    case SIGNUP:
      new_client = first_client_free;
      first_client_free = new_client->next_client_free;
      if ( first_client_free == NULL ) {
        last_client_free = NULL;
      }

      new_client->c_tid = client_tid;
      new_client->next_client_free = NULL;
      new_client->next_client_q = NULL;
      if ( first_client_q == NULL ) {
        first_client_q = new_client;
        last_client_q = new_client;
      } else {
        last_client_q->next_client_q = new_client;
        last_client_q = new_client;
      }
      //bwprintf( COM2, "Task %d signed up to play\r\n", new_client->c_tid );
      
      break;
    case PLAY:
      if( client_tid == player1->c_tid ) {
        p1_action = receive.val;
        switch( p1_action ){
        case ROCK:
          bwprintf( COM2, "Task %d played ROCK\r\n", player1->c_tid );
          break;
        case PAPER:
          bwprintf( COM2, "Task %d played PAPER\r\n", player1->c_tid );
          break;
        case SCISSORS:
          bwprintf( COM2, "Task %d played SCISSORS\r\n", player1->c_tid );
          break;
        }
      } else if( client_tid == player2->c_tid ) {
        p2_action = receive.val;
        switch( p2_action ){
        case ROCK:
          bwprintf( COM2, "Task %d played ROCK\r\n", player2->c_tid );
          break;
        case PAPER:
          bwprintf( COM2, "Task %d played PAPER\r\n", player2->c_tid );
          break;
        case SCISSORS:
          bwprintf( COM2, "Task %d played SCISSORS\r\n", player2->c_tid );
          break;
        }
        if( p1_quit ) {
          reply.type = RESULT;
          reply.val = OPPONENT_QUIT;
          bwprintf( COM2, "Task %d's opponent quit!\r\n", player2->c_tid );
          Reply( player2->c_tid, (char *)&reply, msg_size );
          player2->c_tid = 0;
          if( last_client_free != NULL ) {
            last_client_free->next_client_free = player2;
          } else {
            first_client_free = player2;
          }
          last_client_free = player2;
          p2_action = -1;
          player2 = NULL;
          p1_quit = 0;
          bwputstr( COM2, "Press any key to continue: " );
          bwgetc( COM2 );
          bwputstr( COM2, "\r\n" );
        }
      } else {
        reply.type = ERROR;
        reply.val = PLAY_BEFORE_SIGNUP;
        Reply( client_tid, (char *)&reply, msg_size );
      }

      // Both played
      if( p1_action >= 0 && p2_action >= 0 ) {
        if( p1_action == p2_action ) {
          reply.type = RESULT;
          reply.val = TIE;
          bwprintf( COM2, "TIE!\r\n" );
          Reply( player1->c_tid, (char *)&reply, msg_size );
          Reply( player2->c_tid, (char *)&reply, msg_size );
        }
        else if((p1_action - p2_action + 3) % 3 == 1) {
          reply.type = RESULT;
          reply.val = WIN;
          bwprintf( COM2, "Task %d WINS!\r\n", player1->c_tid );
          Reply( player1->c_tid, (char *)&reply, msg_size );
          reply.val = LOSS;
          Reply( player2->c_tid, (char *)&reply, msg_size );
        }
        else {
          reply.type = RESULT;
          reply.val = LOSS;
          bwprintf( COM2, "Task %d WINS!\r\n", player2->c_tid );
          Reply( player1->c_tid, (char *)&reply, msg_size );
          reply.val = WIN;
          Reply( player2->c_tid, (char *)&reply, msg_size );
        }

        // Put both client structs back on the free list
        player1->c_tid = 0;
        player2->c_tid = 0;
        player1->next_client_free = player2;
        if( last_client_free != NULL ) {
          last_client_free->next_client_free = player1;
        } else {
          first_client_free = player1;
        }
        last_client_free = player2;
        player1 = NULL;
        player2 = NULL;
        p1_action = -1;
        p2_action = -1;
        bwputstr( COM2, "Press any key to continue: " );
        bwgetc( COM2 );
        bwputstr( COM2, "\r\n" );
      }
      break;
    case QUIT:
      debug( "in quit, %d", client_tid );
      if( client_tid == player1->c_tid ) {
          p1_quit = 1;
          bwprintf( COM2, "Task %d Quit!\r\n", client_tid );
          player1->c_tid = 0;
          if( last_client_free != NULL ) {
            last_client_free->next_client_free = player1;
          } else {
            first_client_free = player1;
          }
          last_client_free = player1;
          p1_action = -1;
          player1 = NULL;
      } else if( client_tid == player2->c_tid ) {
          p2_quit = 1;
          bwprintf( COM2, "Task %d Quit!\r\n", client_tid );
          player2->c_tid = 0;
          if( last_client_free != NULL ) {
            last_client_free->next_client_free = player2;
          } else {
            first_client_free = player2;
          }
          last_client_free = player2;
          p2_action = -1;
          player2 = NULL;
          if( p1_quit && p2_quit ){
            p1_quit = 0;
            p2_quit = 0;
          } else {
            reply.type = RESULT;
            reply.val = OPPONENT_QUIT;
            bwprintf( COM2, "Task %d's opponent quit!\r\n", player1->c_tid );
            Reply( player1->c_tid, (char *)&reply, msg_size );
            player1->c_tid = 0;
            if( last_client_free != NULL ) {
              last_client_free->next_client_free = player1;
            } else {
              first_client_free = player1;
            }
            last_client_free = player1;
            p1_action = -1;
            player1 = NULL;
            p2_quit = 0;
          }
          bwputstr( COM2, "Press any key to continue: " );
          bwgetc( COM2 );
          bwputstr( COM2, "\r\n" );
      }
      break;
    default:
      reply.type = ERROR;
      reply.val = INVALID_COMMAND;
      Reply( client_tid, (char *)&reply, msg_size );
      break;
    }

    if( should_quit ) {
      break;
    }

    // No users playing, grab the first user in queue
    if( player1 == NULL && first_client_q != NULL && !p1_quit ) {
      player1 = first_client_q;
      first_client_q = player1->next_client_q;
      player1->next_client_q = NULL;
      if ( first_client_q == NULL ) {
        last_client_q = NULL;
      }
    }
    // We have a first player, now for a second player
    if( player2 == NULL && first_client_q != NULL && !p2_quit ) {
      player2 = first_client_q;
      first_client_q = player2->next_client_q;
      player2->next_client_q = NULL;
      if ( first_client_q == NULL ) {
        last_client_q = NULL;
      }
      reply.type = READY_TO_PLAY;
      reply.val = 0;
      Reply( player1->c_tid, (char *)&reply, msg_size );
      Reply( player2->c_tid, (char *)&reply, msg_size );
      debug( "Players %d, %d ready to play", player1->c_tid, player2->c_tid );
    }
  }
  bwprintf( COM2, "Not enough players. Goodbye!\r\n" );
  Exit( );
}

void rps_client1( ) {
  int i = 0;
  int cmd;
  int timer_val;
  int rps_server_tid = WhoIs( (char *)JOB_RPS );
  rps_msg_t message;
  rps_msg_t reply;
  int msg_size = sizeof(message);
  int my_tid = MyTid( );
  for( ; i < 5; ++i ) {
    message.type = SIGNUP;
    message.val = 0;
    Send( rps_server_tid, (char *)&message, msg_size, (char *)&reply, msg_size );
    if( reply.type == READY_TO_PLAY ) {
      timer_val = get_timer_val( );
      // randomly quit
      if( timer_val % 10 == 0 ) {
        message.type = QUIT;
        message.val = 0;
      } else {
        cmd = timer_val % 3;
        message.type = PLAY;
        message.val = cmd;
      }
      Send( rps_server_tid, (char *)&message, msg_size, (char *)&reply, msg_size );
    } else {
      debug( "SOMETHING WENT WRONG, why can't the client play?" );
    }
  }
  bwprintf( COM2, "Task %d is done playing\r\n", my_tid );
  message.type = QUIT;
  message.val = 0;
  Send( rps_server_tid, (char *)&message, msg_size, (char *)&reply, msg_size );
  Exit( );
}

