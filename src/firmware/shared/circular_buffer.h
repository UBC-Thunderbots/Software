#include <stdbool.h>
#include <stdlib.h>

#include "shared/constants.h"

/**
 * This struct represents the circular buffer properties
 */
typedef struct CircularBuffer CircularBuffer_t;

/**
 * Initialize the circular_buffer struct members
 * @param  cbuffer The circular_buffer
 * @param  size Size of the circular_buffer to set
 */
CircularBuffer_t *circular_buffer_create(CircularBuffer_t *cbuffer, int size);

/**
 * Destroy the circular_buffer, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed to the
 * `create` function
 *
 * @param  cbuffer The circular_buffer to destroy
 */
void circular_buffer_destroy(CircularBuffer_t *cbuffer);

/**
 * Push data into the circular_buffer
 * If full, will overwrite the oldest values
 * @param  cbuffer The circular_buffer
 * @param  data  Value to be inserted into the circular_buffer
 */
void circular_buffer_push(CircularBuffer_t *cbuffer, float data);

/**
 * Retrieve the most recent value in the circular_buffer
 * Ex. 1 is the most recent value, 2 is the 2nd most recent value, etc.
 * @param  cbuffer The circular_buffer
 * @param  index Index of most recent value
 */
float circular_buffer_get_index(CircularBuffer_t *cbuffer, int index);
