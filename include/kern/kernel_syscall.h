#ifndef __KERNEL_SYSCALL_H__
#define __KERNEL_SYSCALL_H__

#include <global.h>

struct global_context_t;

void handle_create( struct global_context_t *gc );

void handle_pass( struct global_context_t *gc );

void handle_exit( struct global_context_t *gc );

void handle_my_tid( struct global_context_t *gc );

void handle_my_parent_tid( struct global_context_t *gc );

void handle_send( struct global_context_t *gc );

void handle_receive( struct global_context_t *gc );

void handle_reply( struct global_context_t *gc );

void handle_await_event( struct global_context_t *gc );

void handle_await_event2( struct global_context_t *gc );

void handle_hwi( struct global_context_t *gc, int hwi_type );

void handle_death( struct global_context_t *gc );

#endif
