#include "app/control.h"

#include "physics/physics.h"
#include "util/util.h"

/**
 * Computes the scaling constant to bring the wheel forces to their maximum
 *
 * Note that this could scale the motor torques *down* if the given forces exceed the
 * physical capabilities of the robot (ex. wheel slip).
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
    float vapp_max = -INFINITY;
    float slip_ratio_min = INFINITY;

    for (long i = 0; i < 4; i++)
    {
        // TODO: better variable names here, some comment(s) would be good to
        Wheel_t* wheel                   = wheels[i];
        const WheelConstants_t constants = app_wheel_getWheelConstants(wheel);
        float force                      = wheel_forces[i];
        float torque   = force * constants.wheel_radius * constants.gear_ratio;
        float curr_rpm = app_wheel_getSpeedRPM(wheel);

        float volt =
            torque * constants.current_per_unit_torque * constants.phase_resistance;
        float back_emf  = curr_rpm * constants.back_emf_per_rpm;
        float appl_volt = fabsf(volt + back_emf);
        float max_app   = fabsf(volt);

        float slip_ratio = constants.max_delta_voltage_before_wheel_slip / max_app;
        if (slip_ratio < slip_ratio_min){
            slip_ratio_min = slip_ratio;
        }
        if (appl_volt > vapp_max)
        {
            vapp_max = appl_volt;
        }
    }

    float emf_ratio_min  = battery_voltage / vapp_max;

    return (emf_ratio_min > slip_ratio_min) ? slip_ratio_min : emf_ratio_min;
}

// TODO: finish this jdoc
/**
 * Compute the maximum TODO
 * @param robot
 * @param linear_accel_x
 * @param linear_accel_y
 * @param angular_accel
 * @return
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
    normed_force[0] = linear_accel_x * robot_constants.linear_mass;
    normed_force[1] = linear_accel_y * robot_constants.linear_mass;
    normed_force[2] =
        angular_accel * robot_constants.rotational_mass * robot_constants.robot_radius;

    float wheel_forces[4];
    force3_to_force4(normed_force, wheel_forces);

    Wheel_t* wheels[4];
    wheels[0] = app_firmware_robot_getFrontLeftWheel(robot);
    wheels[1] = app_firmware_robot_getBackLeftWheel(robot);
    wheels[2] = app_firmware_robot_getBackRightWheel(robot);
    wheels[3] = app_firmware_robot_getFrontRightWheel(robot);

    float battery_voltage =app_firmware_robot_getBatteryVoltage(robot);

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

    float linear_diff_x = linear_accel_x - prev_linear_accel_x;
    float linear_diff_y = linear_accel_y - prev_linear_accel_y;
    float angular_diff  = angular_accel - prev_angular_accel;

    const float jerk_limit = robot_constants.jerk_limit;
    limit(&linear_diff_x, jerk_limit * TICK_TIME);
    limit(&linear_diff_y, jerk_limit * TICK_TIME);
    limit(&angular_diff, jerk_limit / ROBOT_RADIUS * TICK_TIME * 5.0f);

    linear_accel_x = prev_linear_accel_x + linear_diff_x;
    linear_accel_y = prev_linear_accel_y + linear_diff_y;
    angular_accel  = prev_angular_accel + angular_diff;

    prev_linear_accel_x = linear_accel_x;
    prev_linear_accel_y = linear_accel_y;
    prev_angular_accel  = angular_accel;

    // TODO: change linear and rotational masss to mass and moment of inertia

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

void app_control_trackVelocity(FirmwareRobot_t* robot, float linear_velocity_x,
                               float linear_velocity_y, float angular_velocity)
{
    float current_vx               = app_firmware_robot_getVelocityX(robot);
    float current_vy               = app_firmware_robot_getVelocityY(robot);
    float current_angular_velocity = app_firmware_robot_getAngularVelocity(robot);
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

    app_control_applyAccel(robot, current_acceleration[0], current_acceleration[1],
                           angular_acceleration);
}
