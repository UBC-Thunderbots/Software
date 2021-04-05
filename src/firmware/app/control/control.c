#include "firmware/app/control/control.h"

#include "firmware/shared/physics.h"
#include "firmware/shared/util.h"

/**
 * Computes a scaling constant that can be used to maximize wheel force while obeying
 * physical dynamics
 *
 * Note that this could scale the forces *down* if they exceed the physical capabilities
 * of the robot (ex. wheel slip).
 *
 * @param wheels Wheels to compute the maximum torque scaling from
 * @param wheel_forces Forces to apply to each wheel
 * @param battery_voltage The current battery voltage
 *
 * @return The amount by which to scale the force on each wheel to get the maximum
 *         torque possible while maintaining the same torque ratio between wheels
 */
float app_control_getMaximalTorqueScaling(const ForceWheel_t* wheels[4],
                                          const float wheel_forces[4],
                                          float battery_voltage)
{
    float max_effective_motor_voltage = -INFINITY;
    float slip_ratio_min              = INFINITY;

    for (long i = 0; i < 4; i++)
    {
        const ForceWheel_t* wheel             = wheels[i];
        const ForceWheelConstants_t constants = app_force_wheel_getWheelConstants(wheel);
        float force                      = wheel_forces[i];
        float motor_torque =
            force * constants.wheel_radius * constants.wheel_rotations_per_motor_rotation;
        float curr_motor_rpm = app_force_wheel_getMotorSpeedRPM(wheel);

        float resistive_voltage_loss = motor_torque *
                                       constants.motor_current_per_unit_torque *
                                       constants.motor_phase_resistance;
        float back_emf          = curr_motor_rpm * constants.motor_back_emf_per_rpm;
        float effective_voltage = fabsf(resistive_voltage_loss + back_emf);

        float slip_ratio =
            constants.motor_max_voltage_before_wheel_slip / fabsf(resistive_voltage_loss);
        if (slip_ratio < slip_ratio_min)
        {
            slip_ratio_min = slip_ratio;
        }
        if (effective_voltage > max_effective_motor_voltage)
        {
            max_effective_motor_voltage = effective_voltage;
        }
    }

    float emf_ratio_min = battery_voltage / max_effective_motor_voltage;

    return (emf_ratio_min > slip_ratio_min) ? slip_ratio_min : emf_ratio_min;
}

/**
 * Computes the scaling constant for all acceleration components to achieve max abs accel.
 *
 * Note that this could scale the accelerations *down* if the given forces exceed the
 * physical capabilities of the robot (ex. wheel slip).
 *
 * @param robot The robot to compute the acceleration scaling constant for
 * @param linear_accel_x [m/s^2] The linear x acceleration to scale
 * @param linear_accel_y [m/s^2] The linear y acceleration to scale
 * @param angular_accel [rad/s^2] The angular acceleration to scale
 *
 * @return The scaling constant to multiply the all acceleration values by in order to
 *         accelerate the robot as fast as physically possible
 */
float app_control_getMaximalAccelScaling(
    const FirmwareRobot_t* robot, ForceWheel_t* front_left_wheel, ForceWheel_t* front_right_wheel,
    ForceWheel_t* back_left_wheel, ForceWheel_t* back_right_wheel, const float linear_accel_x,
    const float linear_accel_y, float angular_accel)
{
    const RobotConstants_t robot_constants = app_firmware_robot_getRobotConstants(robot);

    // first convert accelerations into consistent units
    // choose units of Force (N)
    float normed_force[3];
    normed_force[0] = linear_accel_x * robot_constants.mass;
    normed_force[1] = linear_accel_y * robot_constants.mass;
    normed_force[2] =
        angular_accel * robot_constants.moment_of_inertia / robot_constants.robot_radius;

    float wheel_forces[4];
    force3_to_force4(normed_force, wheel_forces);

    const ForceWheel_t* wheels[4];
    wheels[0] = front_left_wheel;
    wheels[1] = back_left_wheel;
    wheels[2] = back_right_wheel;
    wheels[3] = front_right_wheel;

    float battery_voltage = app_firmware_robot_getBatteryVoltage(robot);

    return app_control_getMaximalTorqueScaling(wheels, wheel_forces, battery_voltage);
}

void app_control_applyAccel(const FirmwareRobot_t* robot, ForceWheel_t* front_left_wheel,
                            ForceWheel_t* front_right_wheel, ForceWheel_t* back_left_wheel,
                            ForceWheel_t* back_right_wheel, float linear_accel_x,
                            float linear_accel_y, float angular_accel)
{
    const RobotConstants_t robot_constants = app_firmware_robot_getRobotConstants(robot);

    // check for max acceleration in direction of the vel difference
    float scaling = app_control_getMaximalAccelScaling(
        robot, front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel,
        linear_accel_x, linear_accel_y, angular_accel);

    // if the (very naive) 1 tick acceleration violates the physical limits of the robot
    // scale it to maximum
    // if the 1 tick acceleration is below the limit, then leave it
    if (scaling < 1.0f)
    {
        linear_accel_x *= scaling;
        linear_accel_y *= scaling;
        angular_accel *= scaling;
    }

    ControllerState_t* controller_state = app_firmware_robot_getControllerState(robot);
    float prev_linear_accel_x           = controller_state->last_applied_acceleration_x;
    float prev_linear_accel_y           = controller_state->last_applied_acceleration_y;
    float prev_angular_accel = controller_state->last_applied_acceleration_angular;

    float linear_diff_x = linear_accel_x - prev_linear_accel_x;
    float linear_diff_y = linear_accel_y - prev_linear_accel_y;
    float angular_diff  = angular_accel - prev_angular_accel;

    const float jerk_limit                       = robot_constants.jerk_limit;
    const float linear_acceleration_change_limit = robot_constants.jerk_limit * TICK_TIME;
    const float angular_acceleration_change_limit =
        jerk_limit / ROBOT_RADIUS * TICK_TIME * 5.0f;
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
    robot_force[0] = linear_accel_x * robot_constants.mass;
    robot_force[1] = linear_accel_y * robot_constants.mass;
    // input is angular acceleration so mass * Radius * radians/second^2 gives newtons
    robot_force[2] =
        angular_accel * robot_constants.moment_of_inertia / robot_constants.robot_radius;
    float wheel_force[4];
    speed3_to_speed4(robot_force, wheel_force);  // Convert to wheel coordinate system

    app_force_wheel_applyForce(front_left_wheel, wheel_force[0]);
    app_force_wheel_applyForce(front_right_wheel, wheel_force[3]);
    app_force_wheel_applyForce(back_left_wheel, wheel_force[1]);
    app_force_wheel_applyForce(back_right_wheel, wheel_force[2]);
}

void app_control_trackVelocityInRobotFrame(
    FirmwareRobot_t* robot, ForceWheel_t* front_left_wheel, ForceWheel_t* front_right_wheel,
    ForceWheel_t* back_left_wheel, ForceWheel_t* back_right_wheel, float linear_velocity_x,
    float linear_velocity_y, float angular_velocity)
{
    float current_vx               = app_firmware_robot_getVelocityX(robot);
    float current_vy               = app_firmware_robot_getVelocityY(robot);
    float current_angular_velocity = app_firmware_robot_getVelocityAngular(robot);
    float current_orientation      = app_firmware_robot_getOrientation(robot);

    // Rotate the current_velocity vector from the world frame to the robot frame
    float current_velocity[2];
    current_velocity[0] = current_vx;
    current_velocity[1] = current_vy;
    rotate(current_velocity, -current_orientation);

    // This is the "P" term in a PID controller. We essentially do proportional
    // control of our acceleration based on velocity error
    static const float VELOCITY_ERROR_GAIN = 10.0f;

    float desired_acceleration[2];
    desired_acceleration[0] =
        (linear_velocity_x - current_velocity[0]) * VELOCITY_ERROR_GAIN;
    desired_acceleration[1] =
        (linear_velocity_y - current_velocity[1]) * VELOCITY_ERROR_GAIN;

    float angular_acceleration =
        (angular_velocity - current_angular_velocity) * VELOCITY_ERROR_GAIN;

    app_control_applyAccel(robot, front_left_wheel, front_right_wheel, back_left_wheel,
                           back_right_wheel, desired_acceleration[0],
                           desired_acceleration[1], angular_acceleration);
}
