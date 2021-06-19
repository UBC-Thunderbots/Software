#include "firmware/app/control/control.h"

#include "firmware/app/logger/logger.h"
#include "firmware/shared/physics.h"
#include "firmware/shared/util.h"
#include "shared/constants.h"

/**
 * Computes a scaling constant that can be used to maximize wheel force while obeying
 * physical dynamics
 *
 * Note that this could scale the forces *down* if they exceed the physical capabilities
 * of the robot (ex. wheel slip).
 *
 * @param force_wheels Force wheels to compute the maximum torque scaling from
 * @param wheel_forces Forces to apply to each wheel
 * @param battery_voltage The current battery voltage
 *
 * @return The amount by which to scale the force on each wheel to get the maximum
 *         torque possible while maintaining the same torque ratio between wheels
 */
float app_control_getMaximalTorqueScaling(const ForceWheel_t* force_wheels[4],
                                          const float wheel_forces[4],
                                          float battery_voltage, float distance_to_dest)
{
    float leedle_scaling_factor = 1.0f - expf(-10.0f * distance_to_dest);
    return leedle_scaling_factor;
}

/**
 * Computes the scaling constant for all acceleration components to achieve max abs accel.
 *
 * Note that this could scale the accelerations *down* if the given forces exceed the
 * physical capabilities of the robot (ex. wheel slip).
 *
 * @param robot_constants The robot constants representing the robot
 * @param battery_voltage The robot's battery voltage
 * @param force_wheels The robot's force wheels
 * @param linear_accel_x [m/s^2] The linear x acceleration to scale
 * @param linear_accel_y [m/s^2] The linear y acceleration to scale
 * @param angular_accel [rad/s^2] The angular acceleration to scale
 *
 * @return The scaling constant to multiply the all acceleration values by in order to
 *         accelerate the robot as fast as physically possible
 */
float app_control_getMaximalAccelScaling(const RobotConstants_t robot_constants,
                                         float battery_voltage,
                                         const ForceWheel_t* force_wheels[4],
                                         const float linear_accel_x,
                                         const float linear_accel_y, float angular_accel, float distance_to_dest)
{
    // first convert accelerations into consistent units
    // choose units of Force (N)
    float normed_force[3];
    normed_force[0] = linear_accel_x * robot_constants.mass_kg;
    normed_force[1] = linear_accel_y * robot_constants.mass_kg;
    normed_force[2] = angular_accel * robot_constants.moment_of_inertia_kg_m_2 /
                      (float)ROBOT_MAX_RADIUS_METERS;

    float wheel_forces[4];
    shared_physics_force3ToForce4(normed_force, wheel_forces,
                                  robot_constants.front_wheel_angle_deg,
                                  robot_constants.back_wheel_angle_deg);

    return app_control_getMaximalTorqueScaling(force_wheels, wheel_forces,
                                               battery_voltage, distance_to_dest);
}

void app_control_applyAccel(RobotConstants_t robot_constants,
                            ControllerState_t* controller_state, float battery_voltage,
                            ForceWheel_t* force_wheels[4], float linear_accel_x,
                            float linear_accel_y, float angular_accel, float distance_to_dest, float rotation_to_dest)
{
    const ForceWheel_t* wheels[4];
    wheels[0] = force_wheels[0];
    wheels[1] = force_wheels[1];
    wheels[2] = force_wheels[2];
    wheels[3] = force_wheels[3];

    // check for max acceleration in direction of the vel difference
    float scaling =
        app_control_getMaximalAccelScaling(robot_constants, battery_voltage, wheels,
                                           linear_accel_x, linear_accel_y, angular_accel, distance_to_dest);

    float angular_scaling = 1.3f * rotation_to_dest;

    // if the (very naive) 1 tick acceleration violates the physical limits of the robot
    // scale it to maximum
    // if the 1 tick acceleration is below the limit, then leave it
    if (scaling < 1.0f)
    {
        linear_accel_x *= scaling;
        linear_accel_y *= scaling;
    }
    if (angular_scaling < 1.0f)
    {
        angular_accel *= angular_scaling;
    }

    float prev_linear_accel_x = controller_state->last_applied_acceleration_x;
    float prev_linear_accel_y = controller_state->last_applied_acceleration_y;
    float prev_angular_accel  = controller_state->last_applied_acceleration_angular;

    float linear_diff_x = linear_accel_x - prev_linear_accel_x;
    float linear_diff_y = linear_accel_y - prev_linear_accel_y;
    float angular_diff  = angular_accel - prev_angular_accel;

    const float jerk_limit_kg_m_per_s_3 = robot_constants.jerk_limit_kg_m_per_s_3;
    const float linear_acceleration_change_limit =
        robot_constants.jerk_limit_kg_m_per_s_3 * TICK_TIME;
    const float angular_acceleration_change_limit =
        jerk_limit_kg_m_per_s_3 / (float)ROBOT_MAX_RADIUS_METERS * TICK_TIME * 5.0f;
    limit(&linear_diff_x, linear_acceleration_change_limit);
    limit(&linear_diff_y, linear_acceleration_change_limit);
    limit(&angular_diff, angular_acceleration_change_limit);

    linear_accel_x = prev_linear_accel_x + linear_diff_x;
    linear_accel_y = prev_linear_accel_y + linear_diff_y;
    angular_accel  = prev_angular_accel + angular_diff;

    controller_state->last_applied_acceleration_x       = linear_accel_x;
    controller_state->last_applied_acceleration_y       = linear_accel_y;
    controller_state->last_applied_acceleration_angular = angular_accel;

    float robot_force[3];
    robot_force[0] = linear_accel_x * robot_constants.mass_kg;
    robot_force[1] = linear_accel_y * robot_constants.mass_kg;
    // input is angular acceleration so mass * Radius * radians/second^2 gives newtons
    robot_force[2] = angular_accel * robot_constants.moment_of_inertia_kg_m_2 /
                     (float)ROBOT_MAX_RADIUS_METERS;
    float wheel_force[4];
    // Convert to wheel coordinate system
    shared_physics_speed3ToSpeed4(robot_force, wheel_force,
                                  robot_constants.front_wheel_angle_deg,
                                  robot_constants.back_wheel_angle_deg);

    /*TLOG_INFO("%d,%d,%d,%d", (int)(wheel_force[0] * 100), (int)wheel_force[1] * 100,*/
              /*(int)(wheel_force[2] * 100), (int)(wheel_force[3] * 100));*/

    app_force_wheel_applyForce(force_wheels[0], wheel_force[0]);
    app_force_wheel_applyForce(force_wheels[1], wheel_force[1]);
    app_force_wheel_applyForce(force_wheels[2], wheel_force[2]);
    app_force_wheel_applyForce(force_wheels[3], wheel_force[3]);
}
