#include "shared/test_util/test_util.h"

#include "proto/robot_log_msg.pb.h"
#include "software/logger/logger.h"

namespace TestUtil
{
    RobotConstants_t createMockRobotConstants()
    {
        const RobotConstants_t robot_constants = {
            .mass_kg                                = 1.1f,
            .inertial_factor                        = 0.37f,
            .robot_radius_m                         = 0.09f,
            .jerk_limit_kg_m_per_s_3                = 1.4f,
            .front_wheel_angle_deg                  = 1.5f,
            .back_wheel_angle_deg                   = 1.6f,
            .front_of_robot_width_meters            = 1.7f,
            .dribbler_width_meters                  = 1.8f,
            .robot_max_speed_m_per_s                = 1.9f,
            .robot_max_ang_speed_rad_per_s          = 2.0f,
            .robot_max_acceleration_m_per_s_2       = 2.1f,
            .robot_max_ang_acceleration_rad_per_s_2 = 2.2f,
            .indefinite_dribbler_speed_rpm          = 2.3f,
            .max_force_dribbler_speed_rpm           = 2.4f,
            .wheel_radius_meters                    = 2.5f,
            .wheel_rotations_per_motor_rotation     = 2.6f,
            .close_control_speed_m_per_s = 1.0f
        };

        return robot_constants;
    }
};  // namespace TestUtil
