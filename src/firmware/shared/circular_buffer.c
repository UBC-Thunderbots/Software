#include "firmware/shared/circular_buffer.h"

#include <assert.h>

struct CircularBuffer
{
    int head;
    int tail;
    size_t max_size;
    float *buffer_data;
    bool is_full;
};

CircularBuffer_t *circular_buffer_create(size_t size)
{
    CircularBuffer_t *new_cbuffer = malloc(sizeof(CircularBuffer_t));

    new_cbuffer->head        = 0;
    new_cbuffer->tail        = 0;
    new_cbuffer->max_size    = size;
    new_cbuffer->buffer_data = (float *)malloc(sizeof(float) * new_cbuffer->max_size);
    new_cbuffer->is_full     = false;

    return new_cbuffer;
}

void circular_buffer_push(CircularBuffer_t *cbuffer, float data)
{
    cbuffer->buffer_data[cbuffer->head] = data;
    cbuffer->head                       = ++(cbuffer->head) % cbuffer->max_size;
    if (circular_buffer_isFull(cbuffer) == true)
    {
        cbuffer->tail = ++(cbuffer->tail) % cbuffer->max_size;
    }

    // Implementation assumes head and tail are equal when full
    if (cbuffer->head == cbuffer->tail)
    {
        cbuffer->is_full = true;
    }
}

float circular_buffer_getAtIndex(CircularBuffer_t *cbuffer, uint8_t index)
{
    // Check if index is larger than the maximum buffer size
    assert(index <= cbuffer->max_size);

    // Check if enough items are in the buffer for index
    assert(circular_buffer_isFull(cbuffer) == true || index <= cbuffer->head);

    int requested_index = (cbuffer->head % cbuffer->max_size) - index - 1;

    // Check if requestedIndex needs to be looped
    if (requested_index < 0)
    {
        requested_index = cbuffer->max_size + requested_index;
    }
    float res = cbuffer->buffer_data[requested_index];
    return res;
}

float circular_buffer_front(CircularBuffer_t *cbuffer)
{
    // Check if buffer is empty
    assert(circular_buffer_isFull(cbuffer) == true || cbuffer->head > 0);

    float res = circular_buffer_getAtIndex(cbuffer, 0);
    return res;
}

bool circular_buffer_isFull(CircularBuffer_t *cbuffer)
{
    return cbuffer->is_full;
}
