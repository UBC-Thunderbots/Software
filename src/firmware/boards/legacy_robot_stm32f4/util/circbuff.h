#ifndef CIRCBUFF_H
#define CIRCBUFF_H

#include <stdbool.h>

typedef struct
{
    float speed_x;
    float speed_y;
    float speed_angle;
} wheel_speeds_t;

void circbuff_init(wheel_speeds_t queue[], int queueSize);
void add_to_circ_buff(wheel_speeds_t queue[], int queueSize,
                      wheel_speeds_t value);
wheel_speeds_t get_from_circ_buff(wheel_speeds_t queue[], int queueSize,
                                  int index);

#endif  // CIRCBUFF_H
