#include "firmware/shared/circular_buffer.h"

#include <assert.h>

struct CircularBuffer
{
    int head;
    int tail;
    size_t max_size;
    float *buffer_data;
    bool isFull;
};

CircularBuffer_t *circular_buffer_create(size_t size)
{
    CircularBuffer_t *new_cbuffer = malloc(sizeof(CircularBuffer_t));

    new_cbuffer->head        = 0;
    new_cbuffer->tail        = 0;
    new_cbuffer->max_size    = size;
    new_cbuffer->buffer_data = (float *)malloc(sizeof(float) * new_cbuffer->max_size);
    new_cbuffer->isFull      = false;

    return new_cbuffer;
}

void circular_buffer_destroy(CircularBuffer_t *cbuffer)
{
    free(cbuffer);
}

void circular_buffer_push(CircularBuffer_t *cbuffer, float data)
{
    cbuffer->buffer_data[cbuffer->head] = data;
    cbuffer->head                       = (++cbuffer->head) % cbuffer->max_size;
    if (circular_buffer_is_full(cbuffer) == true)
    {
        cbuffer->tail = (++cbuffer->tail) % cbuffer->max_size;
    }

    // Implementation assumes head and tail are equal when full
    if (cbuffer->head == cbuffer->tail)
    {
        cbuffer->isFull = true;
    }
}

float circular_buffer_get_index(CircularBuffer_t *cbuffer, int index)
{
    // Check if index is larger than the maximum buffer size
    assert(index <= cbuffer->max_size);

    // Check if enough items are in the buffer for index
    if (index > cbuffer->head)
    {
        assert(circular_buffer_is_full(cbuffer) != false);
    }

    int requestedIndex = (cbuffer->head % cbuffer->max_size - index);
    if (requestedIndex < 0)
    {
        requestedIndex = cbuffer->max_size + requestedIndex;
    }
    float res = cbuffer->buffer_data[requestedIndex];
    return res;
}

bool circular_buffer_is_full(CircularBuffer_t *cbuffer)
{
    return cbuffer->isFull;
}
