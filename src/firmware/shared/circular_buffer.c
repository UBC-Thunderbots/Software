#include "firmware/shared/circular_buffer.h"
#include "shared/constants.h"
#include <stdlib.h>

// TODO is it better to initialize size or simply do it through CIRCULAR_BUFFER_MAX_NUM_ELEMENTS
void circular_buffer_init(struct circular_buffer *cbuff)
{
    cbuff->head = 0;
    cbuff->tail = 0;
    
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
}

float circular_buffer_get_index(struct circular_buffer *cbuff, int index)
{
    // TODO check if requested index is larger than size
    // TODO what if we request 3rd most recent when there are 2
    // ANS: check if head and tail are equal and go from there

    int requestedIndex = (cbuff->head%CIRCULAR_BUFFER_MAX_NUM_ELEMENTS - index);
    if (requestedIndex < 0)
    {
        requestedIndex = CIRCULAR_BUFFER_MAX_NUM_ELEMENTS + requestedIndex;
    }
    float res = cbuff->bufferData[requestedIndex];
    return res;
}
