#include "software/embedded/motion_control/position_controller.h"

#include <gtest/gtest.h>

#include <memory>

// TODO(#3743): write proper tests after pid constants have been tuned

TEST(PositionControllerTest, BasicTest)
{
    PositionController controller;
    std::shared_ptr trajectory_ptr = std::make_shared<BangBangTrajectory2D>();
    TrajectoryPath trajectory{trajectory_ptr, BangBangTrajectory2D::generator};
    controller.step(Point{}, trajectory, Duration::fromSeconds(1.0),
                    Duration::fromSeconds(0.01));
    controller.reset();
}
