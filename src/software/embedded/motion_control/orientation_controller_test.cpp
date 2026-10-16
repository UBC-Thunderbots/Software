#include "software/embedded/motion_control/orientation_controller.h"

#include <gtest/gtest.h>

#include <tuple>
#include "gtest/gtest.h"

// TODO(#3743): write proper tests after pid constants have been tuned

TEST(OrientationControllerTest, BasicTest)
{
    OrientationController controller;
    BangBangTrajectory1DAngular trajectory;
    controller.step(Angle::zero(), trajectory, Duration::fromSeconds(1.0),
                    Duration::fromSeconds(0.01));
    controller.reset();
}

// !!!

class TestOrientationControllerStep : public testing::TestWithParam<std::tuple<const int>>
{
};

TEST_P(TestOrientationControllerStep, TestOrientationControllerStep) {
    // !!!
}

INSTANTIATE_TEST_SUITE_P(TestTrivial, TestOrientationControllerStep, 
    testing::Values(
        std::tuple{
            // !!!
        }
    )
);

INSTANTIATE_TEST_SUITE_P(TestNormal, TestOrientationControllerStep, 
    testing::Values(
        std::tuple{
            // !!!
        }
    )
);

INSTANTIATE_TEST_SUITE_P(TestEdgeCase, TestOrientationControllerStep, 
    testing::Values(
        std::tuple{
            // !!!
        }
    )
);
