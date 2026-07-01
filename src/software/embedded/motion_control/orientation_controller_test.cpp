#include "software/embedded/motion_control/orientation_controller.h"

#include <gtest/gtest.h>

// TODO(#3743): write proper tests after pid constants have been tuned

TEST(OrientationControllerTest, BasicTest)
{
    OrientationController controller;
    BangBangTrajectory1DAngular trajectory;
    controller.step(Angle::zero(), trajectory, Duration::fromSeconds(1.0),
                    Duration::fromSeconds(0.01));
    controller.reset();
}
