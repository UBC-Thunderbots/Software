// https://github.com/dhess/c-ringbuf/blob/master/ringbuf.c
// https://github.com/dhess/c-ringbuf/blob/master/ringbuf.h
// https://stackoverflow.com/questions/42903600/ring-buffer-on-c

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "shared/constants.h"

struct circular_buffer
{
    float *bufferData;
    int head;
    int tail;
    bool isFull;
};

/**
 * Initialize the circular_buffer struct members
 * @param  cbuff The circular_buffer struct
 */
void circular_buffer_init(struct circular_buffer *cbuff);

/**
 * Free the circular_buffer
 * @param  cbuff The circular_buffer struct
 */
void circular_buffer_free(struct circular_buffer *cbuff);

/**
 * Push data into the circular_buffer
 * If full, will overwrite the oldest values
 * @param  cbuff The circular_buffer struct
 * @param  data  Value to be inserted
 */
void circular_buffer_push(struct circular_buffer *cbuff, float data);

/**
 * Retrieve the most recent value in the circular_buffer
 * Ex. 1 is the most recent value, 2 is the 2nd most recent value, etc.
 * @param  cbuff The circular_buffer struct
 * @param  index Index of most recent value
 */
float circular_buffer_get_index(struct circular_buffer *cbuff, int index);
