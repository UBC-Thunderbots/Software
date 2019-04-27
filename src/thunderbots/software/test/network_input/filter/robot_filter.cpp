/**
 * This file contains the unit tests for the implementation of RobotFilter class
 */

#include "network_input/filter/robot_filter.h"

#inlcude <gtest/gtest.h>
#include <string.h>

TEST(RobotFilterTest,no_match_robot_data_robot_state_expired_test)
{
    Robot robot(1,Point(0,0),Vector(0,0),Angle::ofRadians(0),AngularVelocity::ofRadians(0),Timestamp::fromSeconds(0));
    RobotFilter robot_filter (robot, Duration::fromSeconds(10));
    std::vector<SSLRobotDetection> new_robot_data= {SSLRobotDetection(2,Point(2,0),Angle::ofRadians(1),0.5,Timestamp::fromSeconds(11)) };
    EXPECT_EQ(std::nullopt,robot_filter.getFilteredData(new_robot_data));

}

TEST(RobotFilterTest,no_match_robot_data_robot_state_not_expired_test)
{


}


TEST(RobotFilterTest,one_match_robot_data_robot_state_not_expired_test)
{


}

TEST(RobotFilterTest,multiple_match_robot_data_robot_state_not_expired_test)
{


}
