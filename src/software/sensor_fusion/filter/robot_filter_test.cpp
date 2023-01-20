#include "software/sensor_fusion/filter/robot_filter.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(RobotFilterTest, no_match_robot_data_robot_state_expired_test)
{
    Robot robot(1, Point(0, 0), Vector(0, 0), Angle::fromRadians(0),
                AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0));
    RobotFilter robot_filter(robot, Duration::fromSeconds(10));
    std::vector<RobotDetection> new_robot_data = {
        {2, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromSeconds(11)}};
    EXPECT_EQ(std::nullopt, robot_filter.getFilteredData(new_robot_data));
}

TEST(RobotFilterTest, no_match_robot_data_robot_state_not_expired_test)
{
    Robot robot(1, Point(0, 0), Vector(0, 0), Angle::fromRadians(0),
                AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0));
    RobotFilter robot_filter(robot, Duration::fromSeconds(10));
    std::vector<RobotDetection> new_robot_data = {
        {2, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromSeconds(9)}};
    std::optional<Robot> op_robot(robot);
    EXPECT_EQ(op_robot.value(), robot_filter.getFilteredData(new_robot_data).value());
}


TEST(RobotFilterTest, one_match_robot_data_robot_state_not_expired_test)
{
    Robot robot(1, Point(0, 0), Vector(0, 0), Angle::fromRadians(0),
                AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0));
    RobotFilter robot_filter(robot, Duration::fromSeconds(10));
    std::vector<RobotDetection> new_robot_data = {
        {1, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromSeconds(9)}};
    std::optional<Robot> op_robot;
    op_robot.emplace(Robot(1, Point(2, 0), Vector(2.0 / 9, 0), Angle::fromRadians(1),
                           AngularVelocity::fromRadians(1.0 / 9),
                           Timestamp::fromSeconds(9)));
    EXPECT_EQ(op_robot.value(), robot_filter.getFilteredData(new_robot_data).value());
}

TEST(RobotFilterTest, two_match_robot_data_robot_state_not_expired_test)
{
    Robot robot(1, Point(0, 0), Vector(0, 0), Angle::fromRadians(0),
                AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0));
    RobotFilter robot_filter(robot, Duration::fromSeconds(10));
    std::vector<RobotDetection> new_robot_data = {
        {1, Point(1.5, 0), Angle::fromRadians(0.75), 0.5, Timestamp::fromSeconds(8.5)},
        {1, Point(2.5, 0), Angle::fromRadians(1.25), 0.5, Timestamp::fromSeconds(9.5)}};
    std::optional<Robot> op_robot;
    op_robot.emplace(Robot(1, Point(2, 0), Vector(2.0 / 9, 0), Angle::fromRadians(1),
                           AngularVelocity::fromRadians(1.0 / 9),
                           Timestamp::fromSeconds(9)));
    EXPECT_EQ(op_robot.value(), robot_filter.getFilteredData(new_robot_data).value());
}
