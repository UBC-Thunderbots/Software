#include "circbuff.h"


static int pos = 0;  // Keeps track of the latest element in the queue

void circbuff_init(wheel_speeds_t queue[], int queueSize)
{
    wheel_speeds_t zero;
    zero.speed_x     = 0.0;
    zero.speed_y     = 0.0;
    zero.speed_angle = 0.0;

    for (int i = 0; i < queueSize; i++)
    {
        queue[i] = zero;
    }
}

void add_to_circ_buff(wheel_speeds_t queue[], int queueSize,
                      wheel_speeds_t value)
{
    // If the stop pointer has made it all the way to the start pointer, we have added
    // elements equal to the buffer size So the start element is going to be overwritten
    // and so we should move its pointer ahead by one
    pos++;
    if (pos == queueSize)
    {
        pos = 0;
    }
    queue[pos] = value;
}

wheel_speeds_t get_from_circ_buff(wheel_speeds_t queue[], int queueSize,
                                  int index)
{
    // Get array index of required element
    int idx = pos - index;
    while (idx < 0)
    {
        idx += queueSize;
    }
    // Return element at start pointer
    return queue[idx];
}
