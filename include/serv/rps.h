#ifndef __RPS_H__
#define __RPS_H__

#include <global.h>

/* Message types */
#define SIGNUP              0
#define PLAY                1
#define QUIT                2
#define READY_TO_PLAY       3
#define RESULT              4
#define ERROR               99

/* Errnos */
#define INVALID_COMMAND     -1
#define PLAY_BEFORE_SIGNUP  -2

/* RPS actions */
#define ROCK                0
#define PAPER               1
#define SCISSORS            2

/* RPS returns */
#define TIE                 0
#define WIN                 1
#define LOSS                2
#define OPPONENT_QUIT       3

typedef struct client_t {
  unsigned int c_tid;
  struct client_t *next_client_q;
  struct client_t *next_client_free;
} client_t;

typedef struct rps_msg_t {
  int type;
  int val;
} rps_msg_t;

void rps_server( );
void rps_client1( );
void rps_client2( );
void rps_client3( );

#endif
