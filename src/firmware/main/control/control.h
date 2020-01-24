#ifndef CONTROL_H
#define CONTROL_H

#include "shared/util.h"

#define JERK_LIMIT 40.0f  //(m/s^3)

// Apply wheel force to specific wheels
// TODO: jdoc with direction
void apply_wheel_force_front_right(float force_in_newtons);
void apply_wheel_force_front_left(float force_in_newtons);
void apply_wheel_force_back_right(float force_in_newtons);
void apply_wheel_force_back_left(float force_in_newtons);

#endif
