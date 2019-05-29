#ifndef CONTROL_H
#define CONTROL_H

#include "util/util.h"
//input is acceleration in robot coordinates
//a is in m/s^2
//rot is rotational acceleration in rad/sec^2
void apply_accel(float linear_accel[2], float angular_accel);

//input is force in newtons per wheel to apply the robot
void apply_wheel_force(const float force[4]);

//Takes velocities in global coordinates
//tries to apply accelerations to track it
//essentially a velocity controller.
void track_vel_target(const float linear_vel[2], float angular_vel);

// compute accelerate from a simple bang-bang model
float compute_accel_track_pos_1D(float d_target, float d_cur, float v_cur, float v_max, float a_max);

#endif

