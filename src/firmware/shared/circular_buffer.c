#include "firmware/shared/circular_buffer.h"

#include <assert.h>

struct CircularBuffer
{
    int head;
    int tail;
    int size;
    float *bufferData;
    bool isFull;
};

CircularBuffer_t *circular_buffer_create(CircularBuffer_t *cbuffer, int size)
{
    CircularBuffer_t *new_cbuffer = malloc(sizeof(CircularBuffer_t));

    new_cbuffer->head       = 0;
    new_cbuffer->tail       = 0;
    new_cbuffer->size       = size;
    new_cbuffer->bufferData = (float *)malloc(sizeof(float) * size);
    new_cbuffer->isFull     = false;

    return new_cbuffer;
}

void circular_buffer_free(CircularBuffer_t *cbuffer)
{
    free(cbuffer);
}

void circular_buffer_push(CircularBuffer_t *cbuffer, float data)
{
    cbuffer->bufferData[cbuffer->head] = data;
    cbuffer->head                      = (++cbuffer->head) % cbuffer->size;
    if (cbuffer->isFull == true)
    {
        cbuffer->tail = (++cbuffer->tail) % cbuffer->size;
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
    assert(index > cbuffer->size);

    // Check if enough items are in the buffer for index
    assert(cbuffer->isFull == false && index > cbuffer->head);

    int requestedIndex = (cbuffer->head % cbuffer->size - index);
    if (requestedIndex < 0)
    {
        requestedIndex = cbuffer->size + requestedIndex;
    }
    float res = cbuffer->bufferData[requestedIndex];
    return res;
}
