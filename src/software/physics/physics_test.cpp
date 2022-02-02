#include "software/physics/physics.h"

#include <gtest/gtest.h>

#include <vector>

#include "software/test_util/test_util.h"

TEST(PhysicsUtilTest, test_future_position_opposing_acceleration)
{
    Point initial_position(2, 3);
    Vector initial_velocity(-1, -1);
    Vector acceleration(1, 1);

    Point expected_p(1.5, 2.5);
    Vector expected_v(0, 0);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_p,
        calculateFuturePosition(initial_position, initial_velocity, acceleration,
                                Duration::fromSeconds(1.0)),
        0.001));
}

TEST(PhysicsUtilTest, test_future_velocity_opposing_acceleration)
{
    Point initial_position(2, 3);
    Vector initial_velocity(-1, -1);
    Vector acceleration(1, 1);

    Point expected_p(1.5, 2.5);
    Vector expected_v(0, 0);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_v,
        calculateFutureVelocity(initial_velocity, acceleration,
                                Duration::fromSeconds(1.0)),
        0.001));
}

TEST(PhysicsUtilTest, test_future_position_supporting_acceleration)
{
    Point initial_position(3, 1);
    Vector initial_velocity(1, -2);
    Vector acceleration(1, -2);

    Point expected_p(3.625, -0.25);
    Vector expected_v(1.5, -3);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_p,
        calculateFuturePosition(initial_position, initial_velocity, acceleration,
                                Duration::fromSeconds(0.5)),
        0.001));
}

TEST(PhysicsUtilTest, test_future_velocity_supporting_acceleration)
{
    Point initial_position(3, 1);
    Vector initial_velocity(1, -2);
    Vector acceleration(1, -2);

    Point expected_p(3.625, -0.25);
    Vector expected_v(1.5, -3);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_v,
        calculateFutureVelocity(initial_velocity, acceleration,
                                Duration::fromSeconds(0.5)),
        0.001));
}

TEST(PhysicsUtilTest, test_future_position_sideways_acceleration)
{
    Point initial_position(3, -1.5);
    Vector initial_velocity(1.5, -2);
    Vector acceleration(4, 2);

    Point expected_p(19.25, -0.25);
    Vector expected_v(11.5, 3);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_p,
        calculateFuturePosition(initial_position, initial_velocity, acceleration,
                                Duration::fromSeconds(2.5)),
        0.001));
}

TEST(PhysicsUtilTest, test_future_velocity_sideways_acceleration)
{
    Point initial_position(3, -1.5);
    Vector initial_velocity(1.5, -2);
    Vector acceleration(4, 2);

    Point expected_p(19.25, -0.25);
    Vector expected_v(11.5, 3);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_v,
        calculateFutureVelocity(initial_velocity, acceleration,
                                Duration::fromSeconds(2.5)),
        0.001));
}

class EuclideanToFourWheelTest : public ::testing::Test
{
   protected:
    EuclideanToFourWheelTest() = default;
    EuclideanToFourWheel euclideanToFourWheel;
    WheelSpace_t current_wheel_speeds;
    EuclideanSpace_t target_euclidean_velocity;
    WheelSpace_t expected_wheel_speeds;
};



TEST_F(EuclideanToFourWheelTest, test_target_wheel_speeds_zero)
{
    // test +/right
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {0, 0, 0};
    expected_wheel_speeds     = {0, 0, 0, 0};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclideanToFourWheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                  current_wheel_speeds),
        0.001));
}

TEST_F(EuclideanToFourWheelTest, test_target_wheel_speeds_x)
{
    // test +/right
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {1, 0, 0};
    expected_wheel_speeds     = {-2.2584, -2.2584, 4.0112, 4.0112};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclideanToFourWheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                  current_wheel_speeds),
        0.001));

    // test -/left
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {-1, 0, 0};
    expected_wheel_speeds     = {2.2584, 2.2584, -4.0112, -4.0112};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclideanToFourWheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                  current_wheel_speeds),
        0.001));
}

TEST_F(EuclideanToFourWheelTest, test_target_wheel_speeds_y)
{
    // test +/forwards
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {0, 1, 0};
    expected_wheel_speeds     = {2.1226, -2.1226, -2.7766, 2.7766};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclideanToFourWheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                  current_wheel_speeds),
        0.001));

    // test -/backwards
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {0, -1, 0};
    expected_wheel_speeds     = {-2.1226, 2.1226, 2.7766, -2.7766};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclideanToFourWheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                  current_wheel_speeds),
        0.001));
}

TEST_F(EuclideanToFourWheelTest, test_target_wheel_speeds_w)
{
    // test +/forwards
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {0, 0, 1};
    expected_wheel_speeds     = {0.0918, 0.0918, 0.0885, 0.0885};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclideanToFourWheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                  current_wheel_speeds),
        0.001));

    // test -/backwards
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {0, 0, -1};
    expected_wheel_speeds     = {-0.0918, -0.0918, -0.0885, -0.0885};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclideanToFourWheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                  current_wheel_speeds),
        0.001));
}

TEST_F(EuclideanToFourWheelTest, test_target_wheel_speeds_all)
{
    // test +/forwards
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {1, 1, 1};
    expected_wheel_speeds     = {-0.0440, -4.2893, 1.3231, 6.8763};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclideanToFourWheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                  current_wheel_speeds),
        0.001));
}
