#ifndef __RING_BUF_H__
#define __RING_BUF_H__

#define TYPE_ERROR    -100
#define BUF_OVERFLOW  -101
#define BUF_EMPTY     -102

#define TYPE_CHAR 1 
#define TYPE_INT  4 

typedef union type_arr {
  char * m_arr_char;
  int * m_arr_int;
} type_arr_t;

typedef struct ring_queue {
  int m_size;
  int m_head;
  int m_tail;
  int m_free;
  type_arr_t * m_arr;
} ring_queue_t ;

inline int push_front( int type, ring_queue_t * queue, void * val );

inline int pop_back( int type, ring_queue_t * queue );
inline int pop_front( int type, ring_queue_t * queue );

inline int top_front( int type, ring_queue_t * queue );
inline int top_back( int type, ring_queue_t * queue );

inline int count( ring_queue_t * queue );
inline int empty( ring_queue_t * queue );

#define declare_ring_queue(TYPE, NAME, RING_BUF_SIZE ) \
  compile_assert( RING_BUF_SIZE > 0, invalid_ring_buf_size );\
  ring_queue_t NAME##_queue; \
  NAME##_queue.m_size = RING_BUF_SIZE; \
  NAME##_queue.m_head = 0; \
  NAME##_queue.m_tail = 0; \
  NAME##_queue.m_free = RING_BUF_SIZE; \
  type_arr_t type_arr; \
  TYPE arr[RING_BUF_SIZE]; \
  type_arr.m_arr_char = (char*)&arr; \
  NAME##_queue.m_arr = &type_arr; \
\
  /* return the idx of the inserted value on success, an errno otherwise */ \
  inline int __attribute__((always_inline)) \
  NAME##_push_front( TYPE val ) { \
    return push_front( sizeof( TYPE ), &NAME##_queue, (void *)(&val) ); \
  } \
\
  /* return num of occupied elements in the buffer */ \
  inline int __attribute__((always_inline)) __attribute__((const)) \
  NAME##_count( ) { \
    return count( &NAME##_queue ); \
  } \
\
  inline int __attribute__((always_inline)) \
  NAME##_pop_back( ) { \
    return pop_back( sizeof( TYPE ), &NAME##_queue ); \
  } \
\
  /* return the value of the front element, an errno otherwise */ \
  inline TYPE __attribute__((always_inline)) \
  NAME##_pop_front( ) { \
    return pop_front( sizeof( TYPE ), &NAME##_queue ); \
  } \
\
  inline int __attribute__((always_inline)) __attribute__((const)) \
  NAME##_empty( ) { \
    return empty( &NAME##_queue ); \
  } \
\
  inline int __attribute__((always_inline)) __attribute__((const)) \
  NAME##_top_front( ) { \
    return top_front( sizeof( TYPE ), &NAME##_queue ); \
  } \
\
  inline int __attribute__((always_inline)) __attribute__((const)) \
  NAME##_top_back( ) { \
    return top_back( sizeof( TYPE ), &NAME##_queue ); \
  }

#endif /* __RING_BUF_H__ */
