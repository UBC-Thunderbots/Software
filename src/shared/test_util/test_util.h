#pragma once

#include <gtest/gtest.h>

#include <chrono>

#include "proto/robot_log_msg.nanopb.h"
#include "shared/robot_constants.h"

namespace TestUtil
{
    /**
     * The struct represents the relevant aggregate functions used in testing
     */
    struct AggregateFunctions{
        double avg, maximum, minimum;
    };

    /**
     * Takes a RobotLog proto and creates an INFO level g3log
     *
     * @param robot_log The RobotLog to log
     */
    void handleTestRobotLog(TbotsProto_RobotLog robot_log);

    /**
     * Creates robot constants for a mock robot
     *
     * @return robot constants for a mock robot
     */
    RobotConstants_t createMockRobotConstants();
};  // namespace TestUtil
