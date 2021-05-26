#include "shared/2015_robot_constants.h"

RobotConstants_t create2015RobotConstants(void)
{
    const RobotConstants_t robot_constants = {
        .mass              = 1,
        .moment_of_inertia = 2,
        .robot_radius      = 3,
        .jerk_limit        = 4,
    };
    // RobotConstants_t robot_constants = {
    //    .mass                                              = 1.1f,
    //    .moment_of_inertia                                 = 1.2f,
    //    .robot_radius                                      = 1.3f,
    //    .jerk_limit                                        = 1.4f,
    //    .front_wheel_angle_deg                             = 1.5f,
    //    .back_wheel_angle_deg                              = 1.6f,
    //    .front_of_robot_width_meters                       = 1.7f,
    //    .dribbler_width_meters                             = 1.8f,
    //    .robot_max_speed_meters_per_second                 = 1.9f,
    //    .robot_max_ang_speed_rad_per_second                = 2.0f,
    //    .robot_max_acceleration_meters_per_second_squared  = 2.1f,
    //    .robot_max_ang_acceleration_rad_per_second_squared = 2.2f,
    //    .indefinite_dribbler_speed                         = 2.3f,
    //    .max_force_dribbler_speed                          = 2.4f};
    return robot_constants;
}
