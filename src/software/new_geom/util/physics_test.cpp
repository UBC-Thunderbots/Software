#include "software/new_geom/util/physics.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(PhysicsUtilTest, test_opposing_acceleration)
{
    Point initial_position(2, 3);
    Vector initial_velocity(-1, -1);
    Vector acceleration(1, 1);

    Point expected_p(1.5, 2.5);
    Vector expected_v(0, 0);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_p,
        calculateFuturePosition(initial_position, initial_velocity, acceleration, 1.0),
        0.001));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_v, calculateFutureVelocity(initial_velocity, acceleration, 1.0), 0.001));
}

TEST(PhysicsUtilTest, test_supporting_acceleration)
{
    Point initial_position(3, 1);
    Vector initial_velocity(1, -2);
    Vector acceleration(1, -2);

    Point expected_p(3.625, -0.25);
    Vector expected_v(1.5, -3);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_p,
        calculateFuturePosition(initial_position, initial_velocity, acceleration, 0.5),
        0.001));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_v, calculateFutureVelocity(initial_velocity, acceleration, 0.5), 0.001));
}

TEST(PhysicsUtilTest, test_sideways_acceleration)
{
    Point initial_position(3, -1.5);
    Vector initial_velocity(1.5, -2);
    Vector acceleration(4, 2);

    Point expected_p(19.25, -0.25);
    Vector expected_v(11.5, 3);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_p,
        calculateFuturePosition(initial_position, initial_velocity, acceleration, 2.5),
        0.001));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_v, calculateFutureVelocity(initial_velocity, acceleration, 2.5), 0.001));
}
