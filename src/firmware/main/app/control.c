#include "app/control.h"

#include "physics/physics.h"
//#include "util/util.h"

#define P(x) (int)round(x * 1000)

void limit(float *value, float limiting_value);


/**
 * Computes the scaling constant to bring the wheel forces to their maximum
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
float app_control_getMaximalTorqueScaling(const Wheel_t* wheels[4],
                                          const float wheel_forces[4],
                                          float battery_voltage)
{
    float vapp_max       = -INFINITY;
    float slip_ratio_min = INFINITY;

    for (long i = 0; i < 4; i++)
    {
        // TODO: better variable names here
        Wheel_t* wheel                   = wheels[i];
        const WheelConstants_t constants = app_wheel_getWheelConstants(wheel);
        float force                      = wheel_forces[i];
        float motor_torque =
            force * constants.wheel_radius * constants.wheel_rotations_per_motor_rotation;
        float curr_motor_rpm = app_wheel_getMotorSpeedRPM(wheel);

        float volt = motor_torque * constants.motor_current_per_unit_torque *
                     constants.motor_phase_resistance;
        float back_emf = curr_motor_rpm * constants.motor_back_emf_per_rpm;
        // iprintf("curr_motor_rpm: %d, back_emf_per_rpm: %d \r \n", P(curr_motor_rpm),
        //                P(constants.motor_back_emf_per_rpm));
        float appl_volt = fabsf(volt + back_emf);

        // iprintf("Volt: %d, back_emf: %d \r \n", P(volt), P(back_emf));

        float slip_ratio =
            constants.motor_max_delta_voltage_before_wheel_slip / fabsf(volt);
        if (slip_ratio < slip_ratio_min)
        {
            slip_ratio_min = slip_ratio;
        }
        if (appl_volt > vapp_max)
        {
            vapp_max = appl_volt;
        }
    }

    float emf_ratio_min = battery_voltage / vapp_max;

    // TODO: deleteme
    // iprintf("Battery voltage: %d, vapp_max: %d \r \n", P(battery_voltage),
    // P(vapp_max));
    //    if (emf_ratio_min > slip_ratio_min)
    //    {
    //        iprintf("Limiting based on slip ratio of: %d \r \n", P(slip_ratio_min));
    //    }
    //    else
    //    {
    //        iprintf("Limiting based on emf ratio of: %d \r \n", P(emf_ratio_min));
    //    }

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
float app_control_getMaximalAccelScaling(const FirmwareRobot_t* robot,
                                         const float linear_accel_x,
                                         const float linear_accel_y, float angular_accel)
{
    const RobotConstants_t robot_constants =
        app_firmware_robot_getPhysicalConstants(robot);

    // first convert accelerations into consistent units
    // choose units of Force (N)
    float normed_force[3];
    normed_force[0] = linear_accel_x * robot_constants.mass;
    normed_force[1] = linear_accel_y * robot_constants.mass;
    normed_force[2] =
        angular_accel * robot_constants.moment_of_inertia * robot_constants.robot_radius;

    float wheel_forces[4];
    force3_to_force4(normed_force, wheel_forces);

    Wheel_t* wheels[4];
    wheels[0] = app_firmware_robot_getFrontLeftWheel(robot);
    wheels[1] = app_firmware_robot_getBackLeftWheel(robot);
    wheels[2] = app_firmware_robot_getBackRightWheel(robot);
    wheels[3] = app_firmware_robot_getFrontRightWheel(robot);

    float battery_voltage = app_firmware_robot_getBatteryVoltage(robot);

    return app_control_getMaximalTorqueScaling(wheels, wheel_forces, battery_voltage);
}

void app_control_applyAccel(FirmwareRobot_t* robot, float linear_accel_x,
                            float linear_accel_y, float angular_accel)
{
    const RobotConstants_t robot_constants =
        app_firmware_robot_getPhysicalConstants(robot);

    // check for max acceleration in direction of the vel difference
    float scaling = app_control_getMaximalAccelScaling(robot, linear_accel_x,
                                                       linear_accel_y, angular_accel);

    iprintf("Scaling factor: %d \r \n", P(scaling));

    // if the naive 1 tick acceleration violates the limits of the robot
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

    iprintf("Previous Acceleration: %d, %d, %d \r \n", P(prev_linear_accel_x),
            P(prev_linear_accel_y), P(prev_angular_accel));

    float linear_diff_x = linear_accel_x - prev_linear_accel_x;
    float linear_diff_y = linear_accel_y - prev_linear_accel_y;
    float angular_diff  = angular_accel - prev_angular_accel;

    iprintf("Acceleration diffs: %d, %d, %d \r \n", P(linear_diff_x), P(linear_diff_y),
            P(angular_diff));

    const float jerk_limit = robot_constants.jerk_limit;
    const float linear_jerk_limit = robot_constants.jerk_limit * TICK_TIME;
    iprintf("Jerk limit: %d, TICK_TIME: %d, linear_jerk_limit: %d \r \n", P(robot_constants.jerk_limit), P(TICK_TIME), P(linear_jerk_limit));
    limit(&linear_diff_x, linear_jerk_limit);
    limit(&linear_diff_y, jerk_limit * TICK_TIME);
    limit(&angular_diff, jerk_limit / ROBOT_RADIUS * TICK_TIME * 5.0f);

    iprintf("jerk_limit / ROBOT_RADIUS * TICK_TIME * 5.0f: %d \r \n", P(jerk_limit / ROBOT_RADIUS * TICK_TIME * 5.0f));

    iprintf("limited Acceleration diffs: %d, %d, %d \r \n", P(linear_diff_x),
            P(linear_diff_y), P(angular_diff));

    linear_accel_x = prev_linear_accel_x + linear_diff_x;
    linear_accel_y = prev_linear_accel_y + linear_diff_y;
    angular_accel  = prev_angular_accel + angular_diff;

    iprintf("Jerk Limited Acceleration: %d, %d, %d\r \n", P(linear_accel_x),
            P(linear_accel_y), P(angular_accel));

    controller_state->last_applied_acceleration_x       = linear_accel_x;
    controller_state->last_applied_acceleration_y       = linear_accel_y;
    controller_state->last_applied_acceleration_angular = angular_accel;

    float robot_force[3];
    robot_force[0] = linear_accel_x * robot_constants.mass;
    robot_force[1] = linear_accel_y * robot_constants.mass;
    // input is angular acceleration so mass * Radius * radians/second^2 gives newtons
    robot_force[2] =
        angular_accel * robot_constants.robot_radius * robot_constants.moment_of_inertia;
    float wheel_force[4];
    speed3_to_speed4(robot_force, wheel_force);  // Convert to wheel coordinate syste

    iprintf("Wheel forces: %d, %d, %d, %d \r \n", P(wheel_force[0]), P(wheel_force[1]),
            P(wheel_force[2]), P(wheel_force[3]));

    app_wheel_applyForce(app_firmware_robot_getFrontLeftWheel(robot), wheel_force[0]);
    app_wheel_applyForce(app_firmware_robot_getFrontRightWheel(robot), wheel_force[3]);
    app_wheel_applyForce(app_firmware_robot_getBackLeftWheel(robot), wheel_force[1]);
    app_wheel_applyForce(app_firmware_robot_getBackRightWheel(robot), wheel_force[2]);
}

void app_control_trackVelocity(FirmwareRobot_t* robot, float linear_velocity_x,
                               float linear_velocity_y, float angular_velocity)
{
    float current_vx               = app_firmware_robot_getVelocityX(robot);
    float current_vy               = app_firmware_robot_getVelocityY(robot);
    float current_angular_velocity = app_firmware_robot_getVelocityAngular(robot);
    float current_orientation      = app_firmware_robot_getOrientation(robot);


    // This is the "P" term in a PID controller. We essentially do proportional
    // control of our acceleration based on velocity error
    static const float ACCELERATION_GAIN = 10.0f;

    float current_acceleration[2];
    current_acceleration[0] = (linear_velocity_x - current_vx) * ACCELERATION_GAIN;
    current_acceleration[1] = (linear_velocity_y - current_vy) * ACCELERATION_GAIN;

    // Rotate the acceleration vector from the robot frame to the world frame
    rotate(current_acceleration, -current_orientation);

    float angular_acceleration =
        (angular_velocity - current_angular_velocity) * ACCELERATION_GAIN;

    iprintf("Acceleration In Robot Coordinates: %d, %d, %d \r \n",
            P(current_acceleration[0]), P(current_acceleration[1]),
            P(angular_acceleration));

    app_control_applyAccel(robot, current_acceleration[0], current_acceleration[1],
                           angular_acceleration);
}
