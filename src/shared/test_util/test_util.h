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
    struct AggregateFunctions
    {
        double average, maximum, minimum;
        AggregateFunctions()
        {
            average = 0;
            maximum = 0;
            minimum = DBL_MAX;
        }
    };
};  // namespace TestUtil
