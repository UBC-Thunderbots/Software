#include "shared/test_util/test_util.h"

#include "shared/proto/robot_log_msg.pb.h"
#include "software/logger/logger.h"

namespace TestUtil
{
    void handleTestRobotLog(TbotsProto_RobotLog robot_log)
    {
        LOG(INFO) << "[TEST ROBOT " << robot_log.robot_id << "]["
                  << TbotsProto::LogLevel_Name(
                         static_cast<TbotsProto::LogLevel>(robot_log.log_level))
                  << "]"
                  << "[" << robot_log.file_name << ":" << robot_log.line_number
                  << "]: " << robot_log.log_msg;
    }

    RobotConstants_t createMockRobotConstants()
    {
        const RobotConstants_t robot_constants = {
            .mass_kg                                = 1.1f,
            .moment_of_inertia_kg_m_2               = 1.2f,
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
            .max_force_dribbler_speed_rpm           = 2.4f};

        return robot_constants;
    }
};  // namespace TestUtil
