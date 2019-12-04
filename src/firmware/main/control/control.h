#ifndef CONTROL_H
#define CONTROL_H

#include "util/util.h"
// input is acceleration in robot coordinates
// a is in m/s^2
// rot is rotational acceleration in rad/sec^2
void apply_accel(float linear_accel[2], float angular_accel);

// Apply force in newtons to the wheel with the given index
// TODO: make this private?
void apply_wheel_force(int wheel_index, float force_in_newtons);

// Apply wheel force to specific wheels
void apply_wheel_force_front_right(float force_in_newtons);
void apply_wheel_force_front_left(float force_in_newtons);
void apply_wheel_force_back_right(float force_in_newtons);
void apply_wheel_force_back_left(float force_in_newtons);

// input is force in newtons per wheel to apply the robot
void apply_wheel_force_all_wheels(const float *force);

// Takes velocities in global coordinates
// tries to apply accelerations to track it
// essentially a velocity controller.
void track_vel_target(const float linear_vel[2], float angular_vel);

// compute accelerate from a simple bang-bang model
float compute_accel_track_pos_1D(float d_target, float d_cur, float v_cur, float v_max,
                                 float a_max);

#endif
