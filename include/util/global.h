/* these are the values seen in both kernel space and user space */
#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#define REDBOOT_OFFSET 0x218000
#define NULL ((void *) 0)
#define INT_MAX ((unsigned)0xFFFFFFFF) >> 1 
//#define INT_MIN (signed)0xFFFFFFFF
typedef unsigned int size_t;
typedef enum {
  false = 0,
  true = 1,
} bool;

/* scheduling */
/* NOTE: Lowest priority reserved for idle task */
#define PRIORITY_MAX 16

/* task descriptor related */
#define TD_BIT 7 // TODO: change to 7 (td0 ~ td127) 
#define TD_MAX (1 << TD_BIT) 
#define TID_IDX( id ) ( id & (TD_MAX - 1) )
#define TID_GEN( id ) ( id >> TD_BIT )

/* syscall numbers */
#define SYS_CREATE          1
#define SYS_MY_TID          2
#define SYS_MY_PARENT_TID   3
#define SYS_PASS            4
#define SYS_SEND            5
#define SYS_RECEIVE         6
#define SYS_REPLY           7
#define SYS_AWAIT_EVENT     8
#define SYS_IDLE_PCT_CUM    9 // lol
#define SYS_IDLE_PCT_REC    10
#define SYS_DEATH           98
#define SYS_EXIT            99

/* should not be in this scope, but for the sake of seeing all interrupt values */
#define HWI 100

/* server jobs */
#define SERVER_MAX        6
#define RPS_SERVER        0
#define CLOCK_SERVER      1
#define COM1_OUT_SERVER   2
#define COM1_IN_SERVER    3
#define COM2_OUT_SERVER   4 
#define COM2_IN_SERVER    5


/* timer */
#define TIMER_LOAD_VAL 50800 

/* hardware */
#define CACHE_ON  1
#define CACHE_OFF 0 

/* interrupts */
#define NUM_INTS            5 
#define TIMER3_INT_IND      0
#define COM1_OUT_IND        1
#define COM1_IN_IND         2
#define COM2_OUT_IND        3
#define COM2_IN_IND         4


/* track and trains */
#define TRACK_MAX 140
#define NODE_MAX TRACK_MAX 
#endif
