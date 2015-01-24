/* these are the values seen in both kernel space and user space */
#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#define REDBOOT_OFFSET 0x218000
#define NULL ((void *) 0)

/* task descriptor related */
#define TD_BIT 5 // TODO: change to 7 (127 tds) 
#define TD_MAX (1 << TD_BIT) 
#define TID_IDX( id ) ( id & (TD_MAX - 1) )
#define TID_GEN( id ) ( id >> TD_BIT )

/* syscall numbers */
#define SYS_CREATE 1
#define SYS_MY_TID 2
#define SYS_MY_PARENT_TID 3
#define SYS_PASS 4
#define SYS_SEND 5
#define SYS_KILL 99

#endif

