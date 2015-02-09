#ifndef __NAMESERVER_H__
#define __NAMESERVER_H__

#include <global.h>

#define REGISTER 1
#define WHOIS 2
#define SUCCESS 98
#define ERROR 99

/* Errnos */
// -1 is reserved for nameserver not found
#define INVALID_REQUEST -2
#define MESSAGE_TOO_SHORT -3
#define INVALID_JOB -4
#define SERVER_NOT_FOUND -5

typedef struct nameserver_msg_t {
  int type;
  int val;
} nameserver_msg_t;

void nameserver_main( );

int RegisterAs( char *name );

int WhoIs( char *name );

#endif
