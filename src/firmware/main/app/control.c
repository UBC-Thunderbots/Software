#include "app/control.h"

/**
 * Compute the maximum
 * @param linear_accel_x
 * @param linear_accel_y
 * @param angular_accel
 * @return
 */
float app_control_getMaximalAccelScaling(const float linear_accel_x,
                                         const float linear_accel_y, float angular_accel)
{
    // first convert accelerations into consistent units
    // choose units of Force (N)
    float normed_force[3];
    normed_force[0] = linear_accel[0] * ROBOT_MASS[0];
    normed_force[1] = linear_accel[1] * ROBOT_MASS[1];
    normed_force[2] = angular_accel * ROBOT_MASS[2] * ROBOT_RADIUS;

    float wheel_force[4];
    force3_to_force4(normed_force, wheel_force);
    for (int i = 0; i < 4; ++i)
    {
        wheel_force[i] *= WHEEL_RADIUS * GEAR_RATIO;  // convert to motor torque
    }
    return get_maximal_torque_scaling(wheel_force);
}

void app_control_applyAccel(FirmwareRobot_t* robot, float linear_acceleration_x,
                            float linear_acceleration_y, float angular_accel)
{
    // check for max acceleration in direction of the vel difference
    float scaling = get_maximal_accel_scaling(linear_accel, angular_accel);

    // if the naive 1 tick acceleration violates the limits of the robot
    // scale it to maximum
    // if the 1 tick acceleration is below the limit, then leave it
    if (scaling < 1.0f)
    {
        linear_accel[0] *= scaling;
        linear_accel[1] *= scaling;
        angular_accel *= scaling;
    }
    static float prev_linear_accel0 = 0;
    static float prev_linear_accel1 = 0;
    static float prev_angular_accel = 0;

    float linear_diff0 = linear_accel[0] - prev_linear_accel0;
    float linear_diff1 = linear_accel[1] - prev_linear_accel1;
    float angular_diff = angular_accel - prev_angular_accel;

    limit(&linear_diff0, JERK_LIMIT * TICK_TIME);
    limit(&linear_diff1, JERK_LIMIT * TICK_TIME);
    limit(&angular_diff, JERK_LIMIT / ROBOT_RADIUS * TICK_TIME * 5.0f);

    linear_accel[0] = prev_linear_accel0 + linear_diff0;
    linear_accel[1] = prev_linear_accel1 + linear_diff1;
    angular_accel   = prev_angular_accel + angular_diff;

    prev_linear_accel0 = linear_accel[0];
    prev_linear_accel1 = linear_accel[1];
    prev_angular_accel = angular_accel;

    float robot_force[3];
    robot_force[0] = linear_accel[0] * ROBOT_MASS[0];  // force in x direction
    robot_force[1] = linear_accel[1] * ROBOT_MASS[1];  // force in y direction
    // input is angular acceleration so mass * Radius * radians/second^2 gives newtons
    robot_force[2] = angular_accel * ROBOT_RADIUS * ROBOT_MASS[2];
    float wheel_force[4];
    speed3_to_speed4(robot_force, wheel_force);  // Convert to wheel coordinate syste
    // printf("wheel 0 force: %f", wheel_force[0]);
    // printf("wheel 1 force: %f", wheel_force[1]);
    apply_wheel_force_all_wheels(wheel_force);  // set force
}
