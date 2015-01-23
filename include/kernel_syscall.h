#ifndef __KERNEL_SYSCALL_H__
#define __KERNEL_SYSCALL_H__

#include <global.h>

struct global_context_t;

void handle_create( struct global_context_t *gc );

void handle_pass( struct global_context_t *gc );

void handle_kill( struct global_context_t *gc );

void handle_my_tid( struct global_context_t *gc );

void handle_my_parent_tid( struct global_context_t *gc );

void handle_send( struct global_context_t *gc );

#endif
