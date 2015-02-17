#ifndef __RING_BUF_H__
#define __RING_BUF_H__

#define BUF_OVERFLOW  -1000
#define BUF_EMPTY     -1001

#define declare_ring_buf( NAME, RING_BUF_SIZE ) \
  assert(1, RING_BUF_SIZE >= 0, "ERROR: rint_buf_t initialized with invalid size")\
  typedef struct ring_buf_t {                                                     \
    int m_size;                                                                   \
    int m_arr[RING_BUF_SIZE];                                                     \
    int m_head;                                                                   \
    int m_tail;                                                                   \
  } ring_buf_t ;                                                                  \
                                                                                  \
  ring_buf_t NAME##_buf;                                                          \
  NAME##_buf.m_size = RING_BUF_SIZE;                                              \
  NAME##_buf.m_head = 0;                                                          \
  NAME##_buf.m_tail = RING_BUF_SIZE - 1;                                          \
                                                                                  \
/* return the idx of the inserted value on success, an errno otherwise */         \
  inline int __attribute__((always_inline))                                       \
  NAME##_buf_push_front( int const val ) {                                        \
    if( NAME##_buf.m_head == NAME##_buf.m_tail )                                  \
      return BUF_OVERFLOW;                                                        \
    else {                                                                        \
      int const push_idx = NAME##_buf.m_head;                                     \
      NAME##_buf.m_arr[NAME##_buf.m_head] = val;                                  \
      NAME##_buf.m_head = ( NAME##_buf.m_head + 1 ) % RING_BUF_SIZE;              \
                                                                                  \
      assert( 1, NAME##_buf.m_head >= 0 && NAME##_buf.m_head < RING_BUF_SIZE );   \
      assert( 1, NAME##_buf.m_tail >= 0 && NAME##_buf.m_tail < RING_BUF_SIZE );   \
                                                                                  \
      return push_idx;                                                            \
    }                                                                             \
  }                                                                               \
                                                                                  \
  inline int __attribute__((always_inline)) __attribute__((const))                \
  NAME##_buf_size( ) {                                                            \
    return NAME##_buf.m_size;                                                     \
  }                                                                               \
                                                                                  \
  /* return the value of the front element, an errno otherwise */                 \
  inline int __attribute__((always_inline))                                       \
    NAME##_buf_pop_front( ) {                                                     \
    if( NAME##_buf.m_head == NAME##_buf.m_tail )                                  \
      return BUF_EMPTY;                                                           \
    else {                                                                        \
      int const ret_val = NAME##_buf.m_arr[NAME##_buf.m_head];                    \
      NAME##_buf.m_head = ( NAME##_buf.m_head - 1 ) % RING_BUF_SIZE;              \
                                                                                  \
      assert( 1, NAME##_buf.m_head >= 0 && NAME##_buf.m_head < RING_BUF_SIZE );   \
      assert( 1, NAME##_buf.m_tail >= 0 && NAME##_buf.m_tail < RING_BUF_SIZE );   \
                                                                                  \
      return ret_val;                                                             \
    }                                                                             \
  }                                                                               \
                                                                                  \
  inline int __attribute__((always_inline))                                       \
  NAME##_buf_pop_back( ) {                                                        \
    if( NAME##_buf.m_head == NAME##_buf.m_tail )                                  \
      return BUF_EMPTY;                                                           \
    else {                                                                        \
      int ret_val = NAME##_buf.m_arr[NAME##_buf.m_tail];                          \
      NAME##_buf.m_tail = ( NAME##_buf.m_tail + 1 ) % RING_BUF_SIZE;              \
                                                                                  \
      assert( 1, NAME##_buf.m_head >= 0 && NAME##_buf.m_head < RING_BUF_SIZE );   \
      assert( 1, NAME##_buf.m_tail >= 0 && NAME##_buf.m_tail < RING_BUF_SIZE );   \
                                                                                  \
      return ret_val;                                                             \
    }                                                                             \
  }                                                                               \
                                                                                  \
  inline int __attribute__((always_inline)) __attribute__((const))                \
  NAME##_buf_top_front( ) {                                                       \
    if( NAME##_buf.m_head == NAME##_buf.m_tail )                                  \
      return BUF_EMPTY;                                                           \
    else                                                                          \
      return NAME##_buf.m_arr[NAME##_buf.m_head];                                 \
  }                                                                               \
                                                                                  \
  inline int __attribute__((always_inline)) __attribute__((const))                \
  NAME##_buf_top_back( ) {                                                        \
    if( NAME##_buf.m_head == NAME##_buf.m_tail )                                  \
      return BUF_EMPTY;                                                           \
    else                                                                          \
      return NAME##_buf.m_arr[NAME##_buf.m_tail];                                 \
  }



#endif /* __RING_BUF_H__ */
