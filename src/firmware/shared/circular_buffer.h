#include <stdbool.h>
#include <stdlib.h>

/**
 * This struct represents the circular buffer properties
 */
typedef struct CircularBuffer CircularBuffer_t;

/**
 * Initialize the circular_buffer struct members
 *
 * @param size Size of the circular_buffer to set
 *
 * @pre: `size` must be larger than 0
 *
 * @return A pointer to the created circular_buffer, ownership is given to the caller
 */
CircularBuffer_t *circular_buffer_create(size_t size);

/**
 * Destroy the circular_buffer, freeing any memory allocated for it
 *
 * @param cbuffer The circular_buffer to destroy
 */
void circular_buffer_destroy(CircularBuffer_t *cbuffer);

/**
 * Push data into the circular_buffer
 * If full, will overwrite the oldest value
 *
 * @param cbuffer The circular_buffer
 * @param data  Value to be inserted into the circular_buffer
 */
void circular_buffer_push(CircularBuffer_t *cbuffer, float data);

/**
 * Retrieve a relative recent value in the circular_buffer
 *
 * @param cbuffer The circular_buffer
 * @param index Index relative to the most recent value in the buffer
 *
 * @pre: `index` must be less than the max_size of the circular_buffer
 *
 * @return  Data from the index which is relative to the most recent value in the buffer.
 * Eg. 0 is the most recent value, 1 is the second most recent, etc
 */
float circular_buffer_getAtIndex(CircularBuffer_t *cbuffer, size_t index);

/**
 * Retrieve the most recent value in the circular_buffer
 * Retrieved value is not destroyed or removed from circular_buffer
 *
 * @param cbuffer The circular_buffer
 *
 * @pre: At least one item must be in the circular_buffer
 *
 * @return The most recent value
 */
float circular_buffer_front(CircularBuffer_t *cbuffer);

/**
 * Return if the circular_buffer is full
 *
 * @param cbuffer The circular_buffer
 *
 * @return A boolean indicating if the circular_buffer is full
 */
bool circular_buffer_isFull(CircularBuffer_t *cbuffer);

/**
 * Return if the circular_buffer is empty
 *
 * @param cbuffer The circular_buffer
 *
 * @return A boolean indicating if the circular_buffer is empty
 */
bool circular_buffer_isEmpty(CircularBuffer_t *cbuffer);
