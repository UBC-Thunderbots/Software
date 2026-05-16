#include "shared/robot_constants.h"

#include "shared/constants.h"

RobotConstants create2026RobotConstants()
{
    return {
        .robot_radius_m        = static_cast<float>(ROBOT_MAX_RADIUS_METERS),
        .front_wheel_angle_deg = 32.0f,
        .back_wheel_angle_deg  = 46.0f,

        .front_of_robot_width_meters = 0.11f,
        .dribbler_width_meters       = 0.07825f,

        // Dribbler speeds are negative as that is the direction that sucks the ball in
        .indefinite_dribbler_speed_rpm = -10000,
        .max_force_dribbler_speed_rpm  = -12000,

        // Motor constant
        .motor_max_acceleration_m_per_s_2 = 2.0f,

        // Robot's linear movement constants
        .robot_max_speed_m_per_s            = 2.0f,
        .robot_max_speed_trajectory_m_per_s = 1.5f,
        .robot_max_acceleration_m_per_s_2 = 1.0f,
        .robot_max_deceleration_m_per_s_2 = 0.7f,

        // Robot's angular movement constants
        .robot_max_ang_speed_rad_per_s          = 5.0f,
        .robot_max_ang_speed_trajectory_rad_per_s = 3.6f,
        .robot_max_ang_acceleration_rad_per_s_2 = 2.0f,

        .wheel_radius_meters = 0.03f,

        // Kalman filter variances for robot localizer
        .kalman_process_noise_variance_rad_per_s_4      = 1.0f,
        .kalman_vision_noise_variance_rad_2             = 0.01f,
        .kalman_motor_sensor_noise_variance_rad_per_s_2 = 0.5};
}

RobotConstants create2021RobotConstants()
{
    return {
        .robot_radius_m        = static_cast<float>(ROBOT_MAX_RADIUS_METERS),
        .front_wheel_angle_deg = 32.06f,
        .back_wheel_angle_deg  = 46.04f,

        .front_of_robot_width_meters = 0.11f,
        .dribbler_width_meters       = 0.07825f,

        // Dribbler speeds are negative as that is the direction that sucks the ball in
        .indefinite_dribbler_speed_rpm = -10000,
        .max_force_dribbler_speed_rpm  = -12000,

        // Motor constant
        .motor_max_acceleration_m_per_s_2 = 4.5f,

        // Robot's linear movement constants
        .robot_max_speed_m_per_s          = 3.000f,
        .robot_max_speed_trajectory_m_per_s  = 3.000f,
        .robot_max_acceleration_m_per_s_2 = 3.0f,
        .robot_max_deceleration_m_per_s_2 = 3.0f,

        // Robot's angular movement constants
        .robot_max_ang_speed_rad_per_s          = 10.0f,
        .robot_max_ang_speed_trajectory_rad_per_s = 7.0f,
        .robot_max_ang_acceleration_rad_per_s_2 = 30.0f,

        .wheel_radius_meters = 0.03f,

        // Kalman filter variances for robot localizer
        .kalman_process_noise_variance_rad_per_s_4      = 0.5f,
        .kalman_vision_noise_variance_rad_2             = 0.01f * 0.01f,
        .kalman_motor_sensor_noise_variance_rad_per_s_2 = 0.5f * 0.5f};
}
