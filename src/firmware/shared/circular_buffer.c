#include "firmware/shared/circular_buffer.h"

// TODO is it better to initialize size or simply do it through CIRCULAR_BUFFER_MAX_NUM_ELEMENTS
void circular_buffer_init(struct circular_buffer *cbuff)
{
    cbuff->head = 0;
    cbuff->tail = 0;
    cbuff->isFull = false;
    // Initialize buffer size to CIRCULAR_BUFFER_MAX_NUM_ELEMENTS
    cbuff->bufferData = (float*)malloc(sizeof(float) * CIRCULAR_BUFFER_MAX_NUM_ELEMENTS);
}

void circular_buffer_free(struct circular_buffer *cbuff)
{
    free(cbuff->bufferData);
}

// Note: Implementation assumes head and tail are equal when full
void circular_buffer_push(struct circular_buffer *cbuff, float data)
{
    cbuff->bufferData[cbuff->head%CIRCULAR_BUFFER_MAX_NUM_ELEMENTS] = data;
    cbuff->head++;
    if (cbuff->head > CIRCULAR_BUFFER_MAX_NUM_ELEMENTS)
    {
        cbuff->tail++;
    }
    if (cbuff->head == cbuff->tail)
    {
        cbuff->isFull = true;
    }
}

// Note: Index is starting from 1 for most recent
float circular_buffer_get_index(struct circular_buffer *cbuff, int index)
{
    // Check if index is larger than the maximum buffer size
    if (index > CIRCULAR_BUFFER_MAX_NUM_ELEMENTS)
    {
        return NAN;
    }

    // Check if enough items are in the buffer for index
    if (cbuff->isFull == false && index > cbuff->head)
    {
        return NAN;
    }

    int requestedIndex = (cbuff->head%CIRCULAR_BUFFER_MAX_NUM_ELEMENTS - index);
    if (requestedIndex < 0)
    {
        requestedIndex = CIRCULAR_BUFFER_MAX_NUM_ELEMENTS + requestedIndex;
    }
    float res = cbuff->bufferData[requestedIndex];
    return res;
}
