#ifndef __NAMESERVER_H__
#define __NAMESERVER_H__

#include <global.h>

#define REGISTER 1
#define WHOIS 2
#define SUCCESS 98
#define ERROR 99

/* Errnos */
#define INVALID_REQUEST -1
#define MESSAGE_TOO_SHORT -2
#define INVALID_JOB -3
#define SERVER_NOT_FOUND -4

typedef struct nameserver_msg_t {
  int type;
  int val;
} nameserver_msg_t;

void nameserver_main( );

#endif
