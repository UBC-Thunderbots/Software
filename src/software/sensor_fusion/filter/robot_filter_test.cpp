#include "software/sensor_fusion/filter/robot_filter.h"

#include <gtest/gtest.h>
#include <string.h>

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

// Robot expires when it gets more than 10 frames of only data not matching its own
TEST_F(RobotFilterTest, no_match_robot_data_robot_state_expired_test)
{
    Robot robot(1, Point(0, 0), Vector(0, 0), Angle::fromRadians(0),
                AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0));
    RobotFilter robot_filter(robot);
    std::vector<RobotDetection> new_robot_data;

    // Give it 1 data point so that it doesn't break due to "prev_" variables
    new_robot_data = {
        {1, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromSeconds(0.1)}};
    robot_filter.estimateRobotState(new_robot_data, Timestamp::fromSeconds(0.2));

    // Give it 10 failing frames in total, and it should return nullopt
    constexpr double EXPIRED_FRAME_THRESHOLD_TEST = 10.0;

    for (int i = 1; i < EXPIRED_FRAME_THRESHOLD_TEST + 1; i++)
    {
        new_robot_data = {
            {2, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromSeconds(i)}};
        robot_filter.estimateRobotState(new_robot_data, Timestamp::fromSeconds(i + 0.1));
    }
    new_robot_data = {{2, Point(2, 0), Angle::fromRadians(1), 0.5,
                       Timestamp::fromSeconds(EXPIRED_FRAME_THRESHOLD_TEST + 2)}};
    EXPECT_EQ(
        std::nullopt,
        robot_filter.estimateRobotState(
            new_robot_data, Timestamp::fromSeconds(EXPIRED_FRAME_THRESHOLD_TEST + 2.1)));
}

// Robot does not expire when it gets less than 9 frames of only data not matching its own
TEST_F(RobotFilterTest, no_match_robot_data_robot_state_not_expired_test)
{
    Robot robot(1, Point(0, 0), Vector(0, 0), Angle::fromRadians(0),
                AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0));
    RobotFilter robot_filter(robot);
    std::vector<RobotDetection> new_robot_data;

    // Give it 1 data point so that it doesn't break due to "prev_" variables
    new_robot_data = {
        {1, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromSeconds(0.1)}};
    robot_filter.estimateRobotState(new_robot_data, Timestamp::fromSeconds(0.2));

    // Give it 10 failing frames in total, and it should return nullopt
    constexpr double EXPIRED_FRAME_THRESHOLD_TEST = 9.0;

    for (int i = 1; i < EXPIRED_FRAME_THRESHOLD_TEST + 1; i++)
    {
        new_robot_data = {
            {2, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromSeconds(i)}};
        robot_filter.estimateRobotState(new_robot_data, Timestamp::fromSeconds(i + 0.1));
    }
    new_robot_data              = {{2, Point(2, 0), Angle::fromRadians(1), 0.5,
                                    Timestamp::fromSeconds(EXPIRED_FRAME_THRESHOLD_TEST + 2)}};
    std::optional<Robot> result = robot_filter.estimateRobotState(
        new_robot_data, Timestamp::fromSeconds(EXPIRED_FRAME_THRESHOLD_TEST + 2.1));

    // Result isn't Optional
    ASSERT_TRUE(result.has_value());
    // test
    EXPECT_EQ(result->id(), 1);
    EXPECT_EQ(result->timestamp(),
              Timestamp::fromSeconds(EXPIRED_FRAME_THRESHOLD_TEST + 2.1));
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
                       Timestamp::fromSeconds(3)}};
    std::optional<Robot> result =
        robot_filter.estimateRobotState(new_robot_data, Timestamp::fromSeconds(3.6));

    // Result isn't Optional
    ASSERT_TRUE(result.has_value());
    // Test that the Orientation is less than PI, as it should have crossed over by then
    EXPECT_LT(result->orientation(), Angle::fromRadians(M_PI));
}
