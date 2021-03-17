#pragma once

#include <gtest/gtest.h>

#include <chrono>

#include "shared/proto/robot_log_msg.nanopb.h"

namespace TestUtil
{
    /**
     * Takes a RobotLog proto and creates an INFO level g3log
     *
     * @param robot_log The RobotLog to log
     */
    void handleTestRobotLog(TbotsProto_RobotLog robot_log);
};  // namespace TestUtil
