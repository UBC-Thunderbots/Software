#include "software/embedded/motion_control/position_controller.h"

#include <gtest/gtest.h>

#include <memory>

#include "software/ai/navigator/trajectory/bang_bang_trajectory_2d.h"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/geom/point.h"

class PositionControllerTest : public ::testing::Test
{
   protected:
    PositionControllerTest()
        : trajectory(std::make_shared<BangBangTrajectory2D>()),
          path(TrajectoryPath{trajectory, BangBangTrajectory2D::generator})
    {
    }

    std::shared_ptr<BangBangTrajectory2D> trajectory;
    TrajectoryPath path;
    PositionController controller;

    static constexpr double EPSILON = 1e-4;
};


TEST_F(PositionControllerTest, PurePidControl)
{
    trajectory->generate({0.0, 0.0}, {5.0, 2.0}, {0.0, 0.0}, 1.0, 1e9, 1e9);
    path = TrajectoryPath{trajectory, BangBangTrajectory2D::generator};

    Vector target_velocity =
        controller.step(Point{4.9, 2.1}, path, Duration::fromSeconds(1.0));

    EXPECT_NEAR(2.0 * 0.1, target_velocity.x(), EPSILON);
    EXPECT_NEAR(2.0 * -0.1, target_velocity.y(), EPSILON);
}

TEST_F(PositionControllerTest, FeedforwardControl)
{
    trajectory->generate({0.0, 0.0}, {5.0, 0.0}, {0.0, 0.0}, 1.0, 0.45, 0.45);
    path = TrajectoryPath{trajectory, BangBangTrajectory2D::generator};

    Vector error = {3.0, 2.0};

    Vector target_velocity =
        controller.step(path.getPosition(2.0) + error, path, Duration::fromSeconds(2.0));

    EXPECT_NEAR(path.getVelocity(2.0).x() + 0.8 * -3.0, target_velocity.x(), EPSILON);
    EXPECT_NEAR(0.8 * -2.0, target_velocity.y(), EPSILON);
}

TEST_F(PositionControllerTest, FeedforwardControlNoChange)
{
    trajectory->generate({0.0, 0.0}, {5.0, 0.0}, {0.0, 0.0}, 1.0, 1e9,
                         1e9);  // instant acceleration to 1ms
    path = TrajectoryPath{trajectory, BangBangTrajectory2D::generator};

    Vector target_velocity =
        controller.step(Point{2.0, 0.0}, path, Duration::fromSeconds(2.0));

    EXPECT_NEAR(1.0, target_velocity.x(), EPSILON);
    EXPECT_NEAR(0.0, target_velocity.y(), EPSILON);
}
