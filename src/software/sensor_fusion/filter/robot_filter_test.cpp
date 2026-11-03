#include "software/sensor_fusion/filter/robot_filter.h"

#include <gtest/gtest.h>
#include <string.h>

#include "software/constants.h"
#include "software/test_util/equal_within_tolerance.h"

class RobotFilterTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        default_timestamp = Timestamp::fromSeconds(0);
    }

    Timestamp default_timestamp;
};

// Robot expires when it hasn't received its own data for at least 200 milliseconds.
TEST_F(RobotFilterTest, no_match_robot_data_robot_state_expired_test)
{
    Robot robot(1, Point(0, 0), Vector(0, 0), Angle::fromRadians(0),
                AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0));
    RobotFilter robot_filter(robot);
    std::vector<RobotDetection> new_robot_data;

    // Give it 1 data point so that it doesn't break due to "prev_" variables
    new_robot_data = {
        {1, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromMilliseconds(10)}};
    robot_filter.estimateRobotState(new_robot_data, Timestamp::fromMilliseconds(15));

    new_robot_data = {
        {2, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromMilliseconds(110)}};

    EXPECT_EQ(std::nullopt,
              robot_filter.estimateRobotState(
                  new_robot_data, Timestamp::fromMilliseconds(
                                      ROBOT_DEBOUNCE_DURATION_MILLISECONDS + 115)));
}

// Robot does not expire when it hasn't received its own data for less than 200
// milliseconds.
TEST_F(RobotFilterTest, no_match_robot_data_robot_state_not_expired_test)
{
    Robot robot(1, Point(0, 0), Vector(0, 0), Angle::fromRadians(0),
                AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0));
    RobotFilter robot_filter(robot);
    std::vector<RobotDetection> new_robot_data;

    // Give it 1 data point so that it doesn't break due to "prev_" variables
    new_robot_data = {
        {1, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromMilliseconds(100)}};
    robot_filter.estimateRobotState(new_robot_data, Timestamp::fromMilliseconds(105));

    new_robot_data = {
        {2, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromMilliseconds(110)}};

    std::optional<Robot> result = robot_filter.estimateRobotState(
        new_robot_data,
        Timestamp::fromMilliseconds(ROBOT_DEBOUNCE_DURATION_MILLISECONDS + 100));

    // Result isn't Optional
    ASSERT_TRUE(result.has_value());
    // test
    EXPECT_EQ(result->id(), 1);
    EXPECT_EQ(result->timestamp(),
              Timestamp::fromMilliseconds(ROBOT_DEBOUNCE_DURATION_MILLISECONDS + 100));
}

// tests multiple detections
TEST_F(RobotFilterTest, two_match_robot_data_robot_state_not_expired_test)
{
    Robot robot(1, Point(0, 0), Vector(0, 0), Angle::fromRadians(0),
                AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0));
    RobotFilter robot_filter(robot);

    std::vector<RobotDetection> new_robot_data;

    // Give it 1 data point so that it doesn't break due to "prev_" variables
    new_robot_data = {
        {1, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromSeconds(0.1)}};
    robot_filter.estimateRobotState(new_robot_data, Timestamp::fromSeconds(0.2));

    new_robot_data = {
        {1, Point(1.5, 0), Angle::fromRadians(0.75), 0.5, Timestamp::fromSeconds(8.5)},
        {1, Point(2.5, 0), Angle::fromRadians(1.25), 0.6, Timestamp::fromSeconds(9.5)}};

    std::optional<Robot> result =
        robot_filter.estimateRobotState(new_robot_data, Timestamp::fromSeconds(10));

    // Result isn't Optional
    ASSERT_TRUE(result.has_value());
    // Test that the Orientation went towards 1.25, since the second one has higher
    // confidence.
    EXPECT_GT(result->orientation(), Angle::fromRadians(1));
}

// angle wrapping
TEST_F(RobotFilterTest, large_orientation_angle_wrapping_test)
{
    Robot robot(1, Point(0, 0), Vector(0, 0), Angle::fromDegrees(1.0),
                AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0));
    RobotFilter robot_filter(robot);

    std::vector<RobotDetection> new_robot_data;

    // Give it 1 data point so that it doesn't break due to "prev_" variables
    new_robot_data = {
        {1, Point(2, 0), Angle::fromRadians(0), 0.5, Timestamp::fromSeconds(0.1)}};
    robot_filter.estimateRobotState(new_robot_data, Timestamp::fromSeconds(0.2));

    // make it rotate a lot
    new_robot_data = {{1, Point(2, 0), Angle::fromRadians(M_PI * 2 - 0.2), 0.5,
                       Timestamp::fromSeconds(3)}};
    robot_filter.estimateRobotState(new_robot_data, Timestamp::fromSeconds(3.01));

    // feed it data for another robot, make it predict what it will be
    new_robot_data = {{2, Point(2, 0), Angle::fromRadians(M_PI * 2 - 0.2), 0.5,
                       Timestamp::fromSeconds(3.1)}};
    std::optional<Robot> result =
        robot_filter.estimateRobotState(new_robot_data, Timestamp::fromSeconds(3.11));

    // Result isn't Optional
    ASSERT_TRUE(result.has_value());
    // Test that the Orientation is less than PI, as it should have crossed over by then
    EXPECT_LT(result->orientation(), Angle::fromRadians(M_PI));
}
