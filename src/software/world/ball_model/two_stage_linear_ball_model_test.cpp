#include "software/world/ball_model/two_stage_linear_ball_model.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(TwoStageLinearBallModelTest, test_no_friction)
{
    Point position(3, 5);
    Vector velocity(1, -1);
    TwoStageLinearBallModel model(BallState(position, velocity));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        velocity, model.estimateFutureVelocity(Duration::fromSeconds(1.0)), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        velocity, model.estimateFutureVelocity(Duration::fromSeconds(2.5)), 0.01));
}

TEST(TwoStageLinearBallModelTest, test_rolling_friction)
{
    Point position(3, 5);
    Vector velocity(3, -2);
    TwoStageLinearBallModel model(BallState(position, velocity), 1.3, 2.2, 10);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Vector(1.92, -1.28), model.estimateFutureVelocity(Duration::fromSeconds(1.0)),
        0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Vector(0.30, -0.20), model.estimateFutureVelocity(Duration::fromSeconds(2.5)),
        0.01));
}

TEST(TwoStageLinearBallModelTest, test_sliding_friction)
{
    Point position(3, 5);
    Vector velocity(3, -2);
    TwoStageLinearBallModel model(BallState(position, velocity), 1.3, 2.2, 1);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Vector(1.17, -0.78), model.estimateFutureVelocity(Duration::fromSeconds(1.0)),
        0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Vector(0.0, 0.0), model.estimateFutureVelocity(Duration::fromSeconds(2.5)),
        0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        Vector(0.0, 0.0), model.estimateFutureVelocity(Duration::fromSeconds(5.1)),
        0.01));
}

TEST(TwoStageLinearBallModelTest, test_invalid_args)
{
    Point position(3, 5);
    Vector velocity(3, -2);
    EXPECT_THROW(TwoStageLinearBallModel(BallState(position, velocity), -1.3, 2.2, 1),
                 std::invalid_argument);
    EXPECT_THROW(TwoStageLinearBallModel(BallState(position, velocity), 1.3, -2.2, 1),
                 std::invalid_argument);
    EXPECT_THROW(TwoStageLinearBallModel(BallState(position, velocity), 1.3, 2.2, -1),
                 std::invalid_argument);
}
