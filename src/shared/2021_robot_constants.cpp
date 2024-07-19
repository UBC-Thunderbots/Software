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
        .front_wheel_angle_deg   = 32.06f,
        .back_wheel_angle_deg    = 46.04f,
        // TODO (#2112): update this

        .front_of_robot_width_meters = 0.11f,
        .dribbler_width_meters       = 0.07825f,
        // Dribbler speeds are negative as that is the direction that sucks the ball in
        .indefinite_dribbler_speed_rpm   = -10000.0f,
        .max_force_dribbler_speed_rpm    = -12000.0f,
        .release_ball_dribbler_speed_rpm = 2000.0f,

        // Motor constant
        .motor_max_acceleration_m_per_s_2 = 3.5f,

        // Robot's linear movement constants
        .robot_max_speed_m_per_s                  = 3.000f,
        .ball_placement_wall_max_speed_m_per_s    = 0.3f,
        .ball_placement_retreat_max_speed_m_per_s = 0.3f,
        .dribble_speed_m_per_s                    = 1.5f,
        .robot_max_acceleration_m_per_s_2         = 2.f,
        .robot_max_deceleration_m_per_s_2         = 2.f,

        // Robot's angular movement constants
        .robot_max_ang_speed_rad_per_s          = 10.0f,
        .dribble_max_ang_speed_rad_per_s        = 2.0f,
        .robot_max_ang_acceleration_rad_per_s_2 = 20.0f,

        .wheel_radius_meters                = 0.03f,
        .wheel_rotations_per_motor_rotation = 17.0f / 60.0f};
    return robot_constants;
}
