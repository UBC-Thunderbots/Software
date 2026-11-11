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

// Robot keeps being predicted while it hasn't received its own data for less than
// 1000 milliseconds, and expires once it has gone more than 1000 milliseconds without.
TEST_F(RobotFilterTest, no_match_robot_data_robot_state_expired_test)
{
    Robot robot(1, Point(0, 0), Vector(0, 0), Angle::fromRadians(0),
                AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0));
    RobotFilter robot_filter(robot);
    std::vector<RobotDetection> new_robot_data;

    // Two detections of the robot, 100 milliseconds apart
    new_robot_data = {
        {1, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromMilliseconds(100)}};
    robot_filter.estimateRobotState(new_robot_data, Timestamp::fromMilliseconds(100));

    const unsigned int last_detection_ms = 200;
    new_robot_data = {{1, Point(2, 0), Angle::fromRadians(1), 0.5,
                       Timestamp::fromMilliseconds(last_detection_ms)}};
    robot_filter.estimateRobotState(new_robot_data,
                                    Timestamp::fromMilliseconds(last_detection_ms));

    // From here on, only another robot is detected
    new_robot_data = {{2, Point(2, 0), Angle::fromRadians(1), 0.5,
                       Timestamp::fromMilliseconds(last_detection_ms)}};

    // 500 milliseconds after the last detection, the robot is still predicted
    std::optional<Robot> result = robot_filter.estimateRobotState(
        new_robot_data,
        Timestamp::fromMilliseconds(last_detection_ms +
                                    ROBOT_DEBOUNCE_DURATION_MILLISECONDS / 2));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->id(), 1);

    // Just past the expiry buffer after the last detection, the robot has expired
    EXPECT_EQ(std::nullopt,
              robot_filter.estimateRobotState(
                  new_robot_data,
                  Timestamp::fromMilliseconds(last_detection_ms +
                                              ROBOT_DEBOUNCE_DURATION_MILLISECONDS + 1)));
}

// Robot has not expired yet when exactly 1000 milliseconds have passed since it last
// received its own data
TEST_F(RobotFilterTest, no_match_robot_data_robot_state_expiry_boundary_test)
{
    Robot robot(1, Point(0, 0), Vector(0, 0), Angle::fromRadians(0),
                AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0));
    RobotFilter robot_filter(robot);
    std::vector<RobotDetection> new_robot_data;

    // Two detections of the robot, 100 milliseconds apart
    new_robot_data = {
        {1, Point(2, 0), Angle::fromRadians(1), 0.5, Timestamp::fromMilliseconds(100)}};
    robot_filter.estimateRobotState(new_robot_data, Timestamp::fromMilliseconds(100));

    const unsigned int last_detection_ms = 200;
    new_robot_data = {{1, Point(2, 0), Angle::fromRadians(1), 0.5,
                       Timestamp::fromMilliseconds(last_detection_ms)}};
    robot_filter.estimateRobotState(new_robot_data,
                                    Timestamp::fromMilliseconds(last_detection_ms));

    // From here on, only another robot is detected
    new_robot_data = {{2, Point(2, 0), Angle::fromRadians(1), 0.5,
                       Timestamp::fromMilliseconds(last_detection_ms)}};

    // Exactly at the end of the expiry buffer, the robot is still predicted
    std::optional<Robot> result = robot_filter.estimateRobotState(
        new_robot_data, Timestamp::fromMilliseconds(
                            last_detection_ms + ROBOT_DEBOUNCE_DURATION_MILLISECONDS));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->id(), 1);
    EXPECT_EQ(result->timestamp(),
              Timestamp::fromMilliseconds(last_detection_ms +
                                          ROBOT_DEBOUNCE_DURATION_MILLISECONDS));
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

    // Two detections of the robot, 100 milliseconds apart, turning counterclockwise
    // by 0.2 rad across the wrap point at +-pi
    const Angle before_wrap = Angle::fromRadians(M_PI - 0.1);
    const Angle after_wrap  = Angle::fromRadians(-M_PI + 0.1);

    new_robot_data = {
        {1, Point(2, 0), before_wrap, 0.5, Timestamp::fromMilliseconds(100)}};
    robot_filter.estimateRobotState(new_robot_data, Timestamp::fromMilliseconds(100));

    const unsigned int last_detection_ms = 200;
    new_robot_data              = {{1, Point(2, 0), after_wrap, 0.5,
                                    Timestamp::fromMilliseconds(last_detection_ms)}};
    std::optional<Robot> result = robot_filter.estimateRobotState(
        new_robot_data, Timestamp::fromMilliseconds(last_detection_ms));

    // The estimate should follow the detection across the wrap point, and see a small
    // counterclockwise turn rather than an almost full turn clockwise
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->orientation().minDiff(after_wrap), Angle::fromRadians(0.05));
    EXPECT_GT(result->angularVelocity(), AngularVelocity::zero());

    // From here on, only another robot is detected
    new_robot_data = {{2, Point(2, 0), after_wrap, 0.5,
                       Timestamp::fromMilliseconds(last_detection_ms)}};

    // Within the expiry buffer, the prediction should keep turning counterclockwise
    // past the last detection
    result = robot_filter.estimateRobotState(
        new_robot_data, Timestamp::fromMilliseconds(last_detection_ms + 100));
    ASSERT_TRUE(result.has_value());
    EXPECT_GT((result->orientation() - after_wrap).clamp(), Angle::zero());
    EXPECT_GT(result->angularVelocity(), AngularVelocity::zero());
}
