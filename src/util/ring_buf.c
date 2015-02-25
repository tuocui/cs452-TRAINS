#include "ring_buf.h"
#include "tools.h"

inline int push_front( int type, ring_queue_t * queue, void * val ) {
  debug( "m_free: %d", queue->m_free ); 
  if( queue->m_free == 0 ) { 
    assert( 1, queue->m_head == queue->m_tail, "X" );
    return BUF_OVERFLOW; 
  } 
  else { 
    int push_idx = queue->m_head; 
    if( type == TYPE_INT )
      queue->m_arr->m_arr_int[queue->m_head] = *((int *)val ); 
    else if( type == TYPE_CHAR )
      queue->m_arr->m_arr_char[queue->m_head] = *((char *)val );
    else {
      assert( 1, 1 == 0, "unsupported type %d", type );
      return TYPE_ERROR;
    }

    queue->m_head = ( queue->m_head + 1 ) % queue->m_size; 
    --(queue->m_free); 

    assert( 1, queue->m_head >= 0 && queue->m_head < queue->m_size, "A" );
    assert( 1, queue->m_tail >= 0 && queue->m_tail < queue->m_size, "B" );
    assert( 1, queue->m_free >= 0 , "C");
    return push_idx; 
  } 
}

inline int pop_back( int type, ring_queue_t * queue ) {
  if( queue->m_free == queue->m_size ) {
      assert( 1, queue->m_head == queue->m_tail, "H");
      return BUF_EMPTY;
    }
  else {
    int ret_val;
    if( type == TYPE_INT )
      ret_val = (queue->m_arr->m_arr_int[queue->m_tail]);
    else if ( type == TYPE_CHAR )
      ret_val = (queue->m_arr->m_arr_char[queue->m_tail]);
    else {
      assert( 1, 1 == 0, "unsupported type %d", type );
      return TYPE_ERROR;
    }

    queue->m_tail = ( queue->m_tail + 1 ) % queue->m_size;
    ++(queue->m_free);
    
    assert( 1, queue->m_head >= 0 && queue->m_head < queue->m_size, "I" );
    assert( 1, queue->m_tail >= 0 && queue->m_tail < queue->m_size, "J" );
    
    return ret_val;
    } 
}
  
inline int pop_front( int type, ring_queue_t * queue ) {
  if( queue->m_free == queue->m_size ) { 
      assert( 1, queue->m_head == queue->m_tail, "E");
      return BUF_EMPTY; 
    } 
    else { 
      debug( "m_head: %d", queue->m_head );
      queue->m_head = (queue->m_head - 1 + queue->m_size)%queue->m_size;
      debug( "m_head: %d", queue->m_head );

      int ret_val;
      if( type == TYPE_INT )
        ret_val = queue->m_arr->m_arr_int[queue->m_head];
      else if( type == TYPE_CHAR )
        ret_val = queue->m_arr->m_arr_char[queue->m_head];
      else {
        assert( 1, 1 == 0, "unsupported type %d", type );
        return TYPE_ERROR;
      }

      ++queue->m_free; 

      assert( 1, queue->m_head >= 0 && queue->m_head < queue->m_size, "F" );
      assert( 1, queue->m_tail >= 0 && queue->m_tail < queue->m_size, "G" );

      return ret_val;
    }
}

inline int count( ring_queue_t * queue ) {
  assert( 1, queue->m_size - queue->m_free >= 0 , "D");
  return queue->m_size - queue->m_free; 
}

inline int empty( ring_queue_t * queue ) {
  assert( 1, queue->m_size - queue->m_free >= 0, "H" );
  return ( queue->m_free == queue->m_size ); 
}

inline int top_front( int type, ring_queue_t * queue ) {
  if( queue->m_free == queue->m_size ) {
      assert( 1, queue->m_head == queue->m_tail );
      return BUF_EMPTY;
    } 
  else if( type == TYPE_INT )
    return queue->m_arr->m_arr_int[queue->m_head];  
  else if( type == TYPE_CHAR )
    return queue->m_arr->m_arr_char[queue->m_head];
  else {
    assert( 1, 1 == 0, "unsupported type %d", type );
    return TYPE_ERROR;
  }
}

inline int top_back( int type, ring_queue_t * queue ) {
  if( queue->m_free == queue->m_size ) { 
    assert( 1, queue->m_head == queue->m_tail );
    return BUF_EMPTY; 
  } 
  else if( type == TYPE_INT )
    return queue->m_arr->m_arr_int[queue->m_tail]; 
  else if( type == TYPE_CHAR )
    return queue->m_arr->m_arr_char[queue->m_tail];
  else {
    assert( 1, 1 == 0, "unsupported type %d", type );
    return TYPE_ERROR;
  }
}
