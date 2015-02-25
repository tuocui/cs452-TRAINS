#ifndef __RING_BUF_H__
#define __RING_BUF_H__

#define BUF_OVERFLOW  -100
#define BUF_EMPTY     -101


#define declare_ring_queue( NAME, RING_BUF_SIZE )                                    \
  assert(1, RING_BUF_SIZE >= 0, "ERROR: rint_buf_t initialized with invalid size") \
  typedef struct ring_queue_t {                                                      \
    int m_size;                                                                    \
    int m_arr[RING_BUF_SIZE];                                                      \
    int m_head;                                                                    \
    int m_tail;                                                                    \
    int m_free;                                                                    \
  } ring_queue_t ;                                                                   \
                                                                                   \
  ring_queue_t NAME##_queue;                                                           \
  NAME##_queue.m_size = RING_BUF_SIZE;                                               \
  NAME##_queue.m_head = 0;                                                           \
  NAME##_queue.m_tail = RING_BUF_SIZE - 1;                                           \
  NAME##_queue.m_free = RING_BUF_SIZE;                                               \
                                                                                   \
/* return the idx of the inserted value on success, an errno otherwise */          \
  inline int __attribute__((always_inline))                                        \
  NAME##_queue_push_front( int const val ) {                                         \
      debug( "m_free: %d", NAME##_queue.m_free ); \
    if( NAME##_queue.m_free == 0 ) {                                                 \
      assert( 1, NAME##_queue.m_head == NAME##_queue.m_tail, "X" );                           \
      return BUF_OVERFLOW;                                                         \
    }                                                                              \
    else {                                                                         \
      int const push_idx = NAME##_queue.m_head;                                      \
      NAME##_queue.m_arr[NAME##_queue.m_head] = val;                                   \
      NAME##_queue.m_head = ( NAME##_queue.m_head + 1 ) % RING_BUF_SIZE;               \
      --NAME##_queue.m_free;                                                         \
                                                                                   \
      assert( 1, NAME##_queue.m_head >= 0 && NAME##_queue.m_head < RING_BUF_SIZE, "A" );    \
      assert( 1, NAME##_queue.m_tail >= 0 && NAME##_queue.m_tail < RING_BUF_SIZE, "B" );    \
      assert( 1, NAME##_queue.m_free >= 0 , "C");                                         \
                                                                                   \
      return push_idx;                                                             \
    }                                                                              \
  }                                                                                \
                                                                                   \
  /* return num of occupied elements in the buffer */                              \
  inline int __attribute__((always_inline)) __attribute__((const))                 \
  NAME##_queue_count( ) {                                                            \
    assert( 1, RING_BUF_SIZE - NAME##_queue.m_free >= 0 , "D");                           \
    return RING_BUF_SIZE - NAME##_queue.m_free;                                      \
  }                                                                                \
                                                                                   \
  /* return the value of the front element, an errno otherwise */                  \
  inline int __attribute__((always_inline))                                        \
    NAME##_queue_pop_front( ) {                                                      \
    if( NAME##_queue.m_free == RING_BUF_SIZE ) {                                     \
      assert( 1, NAME##_queue.m_head == NAME##_queue.m_tail, "E");                            \
      return BUF_EMPTY;                                                            \
    }                                                                              \
    else {                                                                         \
      int const ret_val = NAME##_queue.m_arr[NAME##_queue.m_head];                     \
      NAME##_queue.m_head = (NAME##_queue.m_head - 1 + RING_BUF_SIZE)%RING_BUF_SIZE;   \
      ++NAME##_queue.m_free;                                                         \
      debug("%d", NAME##_queue.m_free);\
                                                                                   \
      assert( 1, NAME##_queue.m_head >= 0 && NAME##_queue.m_head < RING_BUF_SIZE, "F" );    \
      assert( 1, NAME##_queue.m_tail >= 0 && NAME##_queue.m_tail < RING_BUF_SIZE, "G" );    \
                                                                                   \
      return ret_val;                                                              \
    }                                                                              \
  }                                                                                \
                                                                                   \
  inline int __attribute__((always_inline))                                        \
  NAME##_queue_pop_back( ) {                                                         \
    if( NAME##_queue.m_free == RING_BUF_SIZE ) {                                     \
      assert( 1, NAME##_queue.m_head == NAME##_queue.m_tail, "H");                         \
      return BUF_EMPTY;                                                            \
    }                                                                              \
    else {                                                                         \
      int ret_val = NAME##_queue.m_arr[NAME##_queue.m_tail];                           \
      NAME##_queue.m_tail = ( NAME##_queue.m_tail + 1 ) % RING_BUF_SIZE;               \
      ++NAME##_queue.m_free;                                                         \
                                                                                   \
      assert( 1, NAME##_queue.m_head >= 0 && NAME##_queue.m_head < RING_BUF_SIZE, "I" );    \
      assert( 1, NAME##_queue.m_tail >= 0 && NAME##_queue.m_tail < RING_BUF_SIZE, "J" );    \
                                                                                   \
      return ret_val;                                                              \
    }                                                                              \
  }                                                                                \
                                                                                   \
  inline int __attribute__((always_inline)) __attribute__((const))                 \
  NAME##_queue_top_front( ) {                                                        \
    if( NAME##_queue.m_free == RING_BUF_SIZE ) {                                     \
      assert( 1, NAME##_queue.m_head == NAME##_queue.m_tail );                         \
      return BUF_EMPTY;                                                            \
    }                                                                              \
    else                                                                           \
      return NAME##_queue.m_arr[NAME##_queue.m_head];                                  \
  }                                                                                \
                                                                                   \
  inline int __attribute__((always_inline)) __attribute__((const))                 \
  NAME##_queue_top_back( ) {                                                         \
    if( NAME##_queue.m_free == RING_BUF_SIZE ) {                                     \
      assert( 1, NAME##_queue.m_head == NAME##_queue.m_tail );                            \
      return BUF_EMPTY;                                                            \
    }                                                                              \
    else                                                                           \
      return NAME##_queue.m_arr[NAME##_queue.m_tail];                                  \
  }                                                                                \



#endif /* __RING_BUF_H__ */
