#ifndef __RING_BUF_H__
#define __RING_BUF_H__

#define TYPE_ERROR    -100
#define BUF_OVERFLOW  -101
#define BUF_EMPTY     -102

#define TYPE_CHAR 1 
#define TYPE_INT  4 
#define TYPE_PTR  8

typedef struct struct_base {
} struct_base_t ;

typedef union type_arr {
  char * m_arr_char;
  int * m_arr_int;
  struct_base_t * m_arr_ptr;
} type_arr_t;

typedef struct ring_queue {
  int m_size;
  int m_head;
  int m_tail;
  int m_free;
  type_arr_t * m_arr;
} ring_queue_t ;

inline int push_front( int type, ring_queue_t * queue, void * val );

inline void * pop_back( int type, ring_queue_t * queue );
inline void * pop_front( int type, ring_queue_t * queue );

inline void * top_front( int type, ring_queue_t * queue );
inline void * top_back( int type, ring_queue_t * queue );

inline int count( ring_queue_t * queue );
inline int empty( ring_queue_t * queue );

#define declare_ring_queue(TYPE, NAME, RING_BUF_SIZE ) \
  compile_assert( RING_BUF_SIZE > 0, NAME##_invalid_ring_buf_size );\
  ring_queue_t NAME##_queue; \
  NAME##_queue.m_size = RING_BUF_SIZE; \
  NAME##_queue.m_head = 0; \
  NAME##_queue.m_tail = 0; \
  NAME##_queue.m_free = RING_BUF_SIZE; \
  type_arr_t NAME##_type_arr; \
  TYPE NAME##_arr[RING_BUF_SIZE]; \
  NAME##_type_arr.m_arr_char = (char*)&NAME##_arr; \
  NAME##_queue.m_arr = &NAME##_type_arr; \
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
  inline TYPE __attribute__((always_inline)) \
  NAME##_pop_back( ) { \
    return *((TYPE *)pop_back( sizeof( TYPE ), &NAME##_queue )); \
  } \
\
  /* return the value of the front element, an errno otherwise */ \
  inline TYPE __attribute__((always_inline)) \
  NAME##_pop_front( ) { \
    return *((TYPE *)pop_front( sizeof( TYPE ), &NAME##_queue )); \
  } \
\
  inline int __attribute__((always_inline)) __attribute__((const)) \
  NAME##_empty( ) { \
    return empty( &NAME##_queue ); \
  } \
\
  inline TYPE __attribute__((always_inline)) __attribute__((const)) \
  NAME##_top_front( ) { \
    return *((TYPE *)top_front( sizeof( TYPE ), &NAME##_queue )); \
  } \
\
  inline TYPE __attribute__((always_inline)) __attribute__((const)) \
  NAME##_top_back( ) { \
    return *((TYPE *)top_back( sizeof( TYPE ), &NAME##_queue )); \
  }

#endif /* __RING_BUF_H__ */
