#include "shared/2021_robot_constants.h"

#include "shared/constants.h"

RobotConstants_t create2021RobotConstants(void)
{
    RobotConstants_t robot_constants = {
        .mass_kg         = 2.5f,   // determined experimentally
        .inertial_factor = 0.37f,  // determined experimentally
        .robot_radius_m  = ROBOT_MAX_RADIUS_METERS,
        // TODO (#2112): update this
        .jerk_limit_kg_m_per_s_3 = 40.0f,
        .front_wheel_angle_deg =
            34.06f,  // The measured angle is 32 degrees, but it only works at 34
        .back_wheel_angle_deg = 46.04f,
        // TODO (#2112): update this
        .front_of_robot_width_meters = 0.11f,
        // TODO (#2112): update this
        .dribbler_width_meters                  = 0.088f,
        .robot_max_speed_m_per_s                = 5.000f,
        .robot_max_wheel_speed_m_per_s          = 5.000f,
        .robot_max_ang_speed_rad_per_s          = 15.00f,
        .robot_max_acceleration_m_per_s_2       = 3.0f,
        .robot_max_ang_acceleration_rad_per_s_2 = 15.0f,
        .robot_max_wheel_acceleration_m_per_s_2 = 10.0f,
        // TODO (#2112): update this
        .indefinite_dribbler_speed_rpm = 10000.0f,
        // TODO (#2112): update this
        .max_force_dribbler_speed_rpm       = -12000.0f,
        .wheel_radius_meters                = 0.03f,
        .wheel_rotations_per_motor_rotation = 17.0f / 60.0f};
    return robot_constants;
}
