#include "app/control.h"

#include "physics/physics.h"
#include "util/util.h"

// TODO: finish this jdoc
/**
 * Compute the maximum
 * @param robot_constants
 * @param linear_accel_x
 * @param linear_accel_y
 * @param angular_accel
 * @return
 */
float app_control_getMaximalAccelScaling(const RobotConstants_t robot_constants,
                                         const float linear_accel_x,
                                         const float linear_accel_y, float angular_accel)
{
    // first convert accelerations into consistent units
    // choose units of Force (N)
    float normed_force[3];
    normed_force[0] = linear_accel_x * robot_constants.linear_mass;
    normed_force[1] = linear_accel_y * robot_constants.linear_mass;
    normed_force[2] =
        angular_accel * robot_constants.rotational_mass * robot_constants.robot_radius;

    float wheel_force[4];
    force3_to_force4(normed_force, wheel_force);
    for (int i = 0; i < 4; ++i)
    {
        // convert to motor torque
        wheel_force[i] *= robot_constants.wheel_radius * robot_constants.gear_ratio;
    }
    return get_maximal_torque_scaling(wheel_force);
}

void app_control_applyAccel(FirmwareRobot_t* robot, float linear_accel_x,
                            float linear_accel_y, float angular_accel)
{
    // TODO: we really should not just arbitrary arrays here...
    const RobotConstants_t robot_constants =
        app_firmware_robot_getPhysicalConstants(robot);

    // check for max acceleration in direction of the vel difference
    float scaling = app_control_getMaximalAccelScaling(robot_constants, linear_accel_x,
                                                       linear_accel_y, angular_accel);

    // if the naive 1 tick acceleration violates the limits of the robot
    // scale it to maximum
    // if the 1 tick acceleration is below the limit, then leave it
    if (scaling < 1.0f)
    {
        linear_accel_x *= scaling;
        linear_accel_y *= scaling;
        angular_accel *= scaling;
    }

    // TODO: static member variables like this are _bad_ we should put it in the
    //       abstraction?? Or maybe at least as a single global member?
    static float prev_linear_accel_x = 0;
    static float prev_linear_accel_y = 0;
    static float prev_angular_accel  = 0;

    float linear_diff0 = linear_accel_x - prev_linear_accel_x;
    float linear_diff1 = linear_accel_y - prev_linear_accel_y;
    float angular_diff = angular_accel - prev_angular_accel;

    const float jerk_limit = robot_constants.jerk_limit;
    limit(&linear_diff0, jerk_limit * TICK_TIME);
    limit(&linear_diff1, jerk_limit * TICK_TIME);
    limit(&angular_diff, jerk_limit / ROBOT_RADIUS * TICK_TIME * 5.0f);

    linear_accel_x = prev_linear_accel_x + linear_diff0;
    linear_accel_y = prev_linear_accel_y + linear_diff1;
    angular_accel  = prev_angular_accel + angular_diff;

    prev_linear_accel_x = linear_accel_x;
    prev_linear_accel_y = linear_accel_y;
    prev_angular_accel  = angular_accel;

    float robot_force[3];
    robot_force[0] = linear_accel_x * robot_constants.linear_mass;
    robot_force[1] = linear_accel_y * robot_constants.linear_mass;
    // input is angular acceleration so mass * Radius * radians/second^2 gives newtons
    robot_force[2] =
        angular_accel * robot_constants.robot_radius * robot_constants.rotational_mass;
    float wheel_force[4];
    speed3_to_speed4(robot_force, wheel_force);  // Convert to wheel coordinate syste

    app_wheel_applyForce(app_firmware_robot_getFrontLeftWheel(robot), wheel_force[0]);
    app_wheel_applyForce(app_firmware_robot_getFrontRightWheel(robot), wheel_force[3]);
    app_wheel_applyForce(app_firmware_robot_getBackLeftWheel(robot), wheel_force[1]);
    app_wheel_applyForce(app_firmware_robot_getBackRightWheel(robot), wheel_force[2]);
}
