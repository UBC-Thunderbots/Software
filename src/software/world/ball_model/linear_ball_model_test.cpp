#include "software/world/ball_model/linear_ball_model.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(LinearBallModelTest, test_no_friction)
{
    Point position(3, 5);
    Vector velocity(1, -1);
    LinearBallModel model(BallState(position, velocity));
    BallState expected_1(Point(4, 4), velocity);
    BallState expected_2(Point(5.5, 2.5), velocity);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_1.position(), model.estimateFutureState(1.0).position(), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_1.velocity(), model.estimateFutureState(1.0).velocity(), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_2.position(), model.estimateFutureState(2.5).position(), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_2.velocity(), model.estimateFutureState(2.5).velocity(), 0.01));
}

TEST(LinearBallModelTest, test_rolling_friction)
{
    LinearBallModel::FrictionParameters fp{.rolling_friction_acceleration   = 1.3,
                                           .sliding_friction_acceleration   = -2.2,
                                           .rolling_sliding_speed_threshold = 10};
    Point position(3, 5);
    Vector velocity(3, -2);
    LinearBallModel model(BallState(position, velocity), fp);
    BallState expected_1(Point(5.46, 3.36), Vector(1.92, -1.28));
    BallState expected_2(Point(7.12, 2.25), Vector(0.30, -0.20));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_1.position(), model.estimateFutureState(1.0).position(), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_1.velocity(), model.estimateFutureState(1.0).velocity(), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_2.position(), model.estimateFutureState(2.5).position(), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_2.velocity(), model.estimateFutureState(2.5).velocity(), 0.01));
}

TEST(LinearBallModelTest, test_sliding_friction)
{
    LinearBallModel::FrictionParameters fp{.rolling_friction_acceleration   = 1.3,
                                           .sliding_friction_acceleration   = -2.2,
                                           .rolling_sliding_speed_threshold = 1};
    Point position(3, 5);
    Vector velocity(3, -2);
    LinearBallModel model(BallState(position, velocity), fp);
    BallState expected_1(Point(5.08, 3.61), Vector(1.17, -0.78));
    BallState expected_2(Point(5.59, 3.27), Vector(0.0, 0.0));
    BallState expected_3(Point(5.59, 3.27), Vector(0.0, 0.0));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_1.position(), model.estimateFutureState(1.0).position(), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_1.velocity(), model.estimateFutureState(1.0).velocity(), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_2.position(), model.estimateFutureState(2.5).position(), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_2.velocity(), model.estimateFutureState(2.5).velocity(), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_3.position(), model.estimateFutureState(5.1).position(), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_3.velocity(), model.estimateFutureState(5.1).velocity(), 0.01));
}
