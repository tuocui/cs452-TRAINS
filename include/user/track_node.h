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

struct track_node;

typedef struct track_edge {
  struct track_edge *reverse;
  struct track_node *src, *dest;
  int dist;             /* in millimetres */
} track_edge_t;

typedef struct track_node {
  const char *name;
  node_type_t type;
  int num;              /* sensor or switch number */
  struct track_node *reverse;  /* same location, but opposite direction */
  struct track_edge edge[2];
} track_node_t;

#endif

