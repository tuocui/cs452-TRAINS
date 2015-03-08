#include "ring_buf.h"
#include "global.h"
#include "tools.h"

inline int push_front( int type, ring_queue_t * queue, void * val ) {
  debug( "m_free: %d", queue->m_free ); 
  if( queue->m_free == 0 ) { 
    assertm( 1, queue->m_head == queue->m_tail, "X" );
    return BUF_OVERFLOW; 
  } 
  else { 
    int push_idx = queue->m_head; 
    if( type == TYPE_INT )
      queue->m_arr->m_arr_int[queue->m_head] = *((int *)val ); 
    else if( type == TYPE_CHAR )
      queue->m_arr->m_arr_char[queue->m_head] = *((char *)val );
    else if( type == TYPE_PTR )
      queue->m_arr->m_arr_ptr[queue->m_head] = *((struct_base_t *)val );
    else {
      assertm( 1, 1 == 0, "unsupported type %d", type );
      return TYPE_ERROR;
    }

    queue->m_head = ( queue->m_head + 1 ) % queue->m_size; 
    --(queue->m_free); 

    assertm( 1, queue->m_head >= 0 && queue->m_head < queue->m_size, "A" );
    assertm( 1, queue->m_tail >= 0 && queue->m_tail < queue->m_size, "B" );
    assertm( 1, queue->m_free >= 0 , "C");
    return push_idx; 
  } 
}

inline void * pop_back( int type, ring_queue_t * queue ) {
  if( queue->m_free == queue->m_size ) {
      assertm( 1, queue->m_head == queue->m_tail, "H");
      assertm( 2, 1 == 0, "ring_queue is empty" );
      return NULL;
    }
  else {
    void * ret_val;
    if( type == TYPE_INT )
      ret_val = (void*)&(queue->m_arr->m_arr_int[queue->m_tail]);
    else if ( type == TYPE_CHAR )
      ret_val = (void*)&(queue->m_arr->m_arr_char[queue->m_tail]);
    else if ( type == TYPE_PTR )
      ret_val = (void*)&(queue->m_arr->m_arr_ptr[queue->m_tail]);
    else {
      assertm( 1, 1 == 0, "unsupported type %d", type );
      return NULL;
    }

    queue->m_tail = ( queue->m_tail + 1 ) % queue->m_size;
    ++(queue->m_free);
    
    assertm( 1, queue->m_head >= 0 && queue->m_head < queue->m_size, "I" );
    assertm( 1, queue->m_tail >= 0 && queue->m_tail < queue->m_size, "J" );
    
    return ret_val;
    } 
}
  
inline void * pop_front( int type, ring_queue_t * queue ) {
  if( queue->m_free == queue->m_size ) { 
      assertm( 1, queue->m_head == queue->m_tail, "E");
      assertm( 2, 1 == 0, "ring_queue is empty" );
      return NULL; 
    } 
    else { 
      debug( "m_head: %d", queue->m_head );
      queue->m_head = ( queue->m_head - 1 + queue->m_size ) % queue->m_size;
      debug( "m_head: %d", queue->m_head );

      void * ret_val;
      if( type == TYPE_INT )
        ret_val = (void*)&(queue->m_arr->m_arr_int[queue->m_head]);
      else if( type == TYPE_CHAR )
        ret_val = (void*)&(queue->m_arr->m_arr_char[queue->m_head]);
      else if( type == TYPE_PTR )
        ret_val = (void*)&(queue->m_arr->m_arr_ptr[queue->m_head]);
      else {
        assertm( 1, 1 == 0, "unsupported type %d", type );
        return NULL;
      }

      ++queue->m_free; 

      assertm( 1, queue->m_head >= 0 && queue->m_head < queue->m_size, "F" );
      assertm( 1, queue->m_tail >= 0 && queue->m_tail < queue->m_size, "G" );

      return ret_val;
    }
}

inline int count( ring_queue_t * queue ) {
  assertm( 1, queue->m_size - queue->m_free >= 0 , "D");
  return queue->m_size - queue->m_free; 
}

inline int empty( ring_queue_t * queue ) {
  assertm( 1, queue->m_size - queue->m_free >= 0, "H" );
  return ( queue->m_free == queue->m_size ); 
}

inline void * top_front( int type, ring_queue_t * queue ) {
  if( queue->m_free == queue->m_size ) {
    assert( 1, queue->m_head == queue->m_tail );
    assertm( 2, 1 == 0, "ring_queue is empty" );
    return NULL;
  } 
  else if( type == TYPE_INT )
    return (void*)&(queue->m_arr->m_arr_int[queue->m_head]);  
  else if( type == TYPE_CHAR )
    return (void*)&(queue->m_arr->m_arr_char[queue->m_head]);
  else if( type == TYPE_PTR )
    return (void*)&(queue->m_arr->m_arr_ptr[queue->m_head]);
  else {
    assertm( 1, 1 == 0, "unsupported type %d", type );
    return NULL;
  }
}

inline void * top_back( int type, ring_queue_t * queue ) {
  if( queue->m_free == queue->m_size ) { 
    assert( 1, queue->m_head == queue->m_tail );
    assertm( 2, 1 == 0, "ring_queue is empty" );
    return NULL; 
  } 
  else if( type == TYPE_INT )
    return (void*)&(queue->m_arr->m_arr_int[queue->m_tail]); 
  else if( type == TYPE_CHAR )
    return (void*)&(queue->m_arr->m_arr_char[queue->m_tail]);
  else if( type == TYPE_PTR )
    return (void*)&(queue->m_arr->m_arr_ptr[queue->m_tail]);
  else {
    assertm( 1, 1 == 0, "unsupported type %d", type );
    return NULL;
  }
}
