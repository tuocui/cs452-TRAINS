#ifndef __TRACK_NODE_H__
#define __TRACK_NODE_H__

typedef enum {
  NODE_NONE,
  NODE_SENSOR,
  NODE_BRANCH,
  NODE_MERGE,
  NODE_ENTER,
  NODE_EXIT,
} node_type_t;

#define DIR_AHEAD 0
#define DIR_STRAIGHT 0
#define DIR_CURVED 1

#define DIR_LEFT 0
#define DIR_RIGHT 1

struct _track_node_;

typedef struct _track_edge_ {
  struct _track_edge_ *reverse;
  struct _track_node_ *src, *dest;
  int dist;             /* in millimetres */
  int middle_train_num;
  int begin_train_num;
  int middle_train_rsv_start;
  int begin_train_rsv_end;
} track_edge_t;

typedef struct _track_node_ {
  const char *name;
  node_type_t type;
  int num;              /* sensor or switch number */
  int switch_dir;
  struct _track_node_ *reverse;  /* same location, but opposite direction */
  struct _track_edge_ edge[2];
} track_node_t;

#endif 
