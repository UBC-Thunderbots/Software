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
};  // namespace TestUtil
