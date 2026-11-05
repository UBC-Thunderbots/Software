#include "gtest/gtest.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/embedded/motion_control/orientation_controller.h"
#include "software/embedded/motion_control/pid_controller.h"
#include "software/geom/angle.h"
#include "software/time/duration.h"

#include <gtest/gtest.h>

namespace 
{
    const PidController<double> _pid = OrientationController{}.getPid();

    const Angle pi_over_4 = Angle::quarter() / 2.0;
    const double proportional = _pid.getProportional();
    const double integral = _pid.getDerivative();
    const double derivative = _pid.getIntegral();
}

struct TestInputStepTrivial 
{
    const AngularVelocity expected;
    const Angle orientation;
    const BangBangTrajectory1DAngular trajectory;
};

class TestStepTrivial : public testing::TestWithParam<TestInputStepTrivial>
{
};

/** @brief Parametrized test for checking basic functionality. */
TEST_P(TestStepTrivial, TestOrientationControllerStep) 
{
    const TestInputStepTrivial& input = GetParam();

    const Duration time = Duration::fromSeconds(1.0);

    const AngularVelocity actual = OrientationController().step(input.orientation, input.trajectory, time, time);

    EXPECT_EQ(input.expected, actual);
}

/** @brief Creates a trajectory. 
 * @details Passes parameters directly to a constructor of BangBangTrajectory1DAngular.
 */
BangBangTrajectory1DAngular create_trajectory_path(const Angle& final_orienation);

/** @brief Sanity check. */
INSTANTIATE_TEST_SUITE_P(TestStepSanity, TestStepTrivial, 
    testing::Values(
        /** @brief Test for zero angle. */
        TestInputStepTrivial
        {
            .expected=AngularVelocity{Angle::zero()},
            .orientation=Angle::zero(),
            .trajectory=create_trajectory_path(Angle::zero())
        },
        /** @brief Test for nonzero angle. */
        TestInputStepTrivial
        {
            .expected=AngularVelocity{Angle::zero()},
            .orientation=Angle::half(),
            .trajectory=create_trajectory_path(Angle::half())
        }
    )
);

/** @brief Test if robot can spin in both directions. */
INSTANTIATE_TEST_SUITE_P(TestStepGenericMovement, TestStepTrivial,
    testing::Values(
        /** @brief Test if robot can spin clockwise. */
        TestInputStepTrivial
        {
            .expected=AngularVelocity{Angle::half() * proportional},
            .orientation=Angle::half(),
            .trajectory=create_trajectory_path(Angle::zero())
        },
        /** @brief Test if robot can spin counterclockwise. */
        TestInputStepTrivial
        {
            .expected=AngularVelocity{-Angle::quarter() * proportional},
            .orientation=Angle::quarter(),
            .trajectory=create_trajectory_path(Angle::zero())
        }
    )
);

BangBangTrajectory1DAngular create_trajectory_path(const Angle& final_orienation) 
{
    const AngularVelocity maximum_angular_velocity{Angle::half()};
    const AngularAcceleration maximum_angular_acceleration{Angle::half()};
    const AngularAcceleration maximum_angular_deceleration{Angle::half()};

    const Angle initial_orienation{Angle::zero()};
    const AngularVelocity initial_angular_velocity{Angle::zero()};

    return BangBangTrajectory1DAngular{initial_orienation, final_orienation, initial_angular_velocity, maximum_angular_velocity, maximum_angular_acceleration, maximum_angular_deceleration};
}

struct TestInputStepErrorCorrection
{
public:
    const AngularVelocity expected;
    const Angle orienation;
};

class TestStepErrorCorrectionOnce : public testing::TestWithParam<TestInputStepErrorCorrection>
{
};

/** @brief Parametrized test for a single instance of error correction.
 * @details From 0rads^-1, we accelerate at (pi/8)rads^-2 for 2s to reach (pi/4)rads^-1,
 * and decelerate at (pi/8)rad^-2 for 2s to reach 0rads^-1, rotating (pi/4)rad in 4s.
 * @note We rotate counterclockwise.
 */
TEST_P(TestStepErrorCorrectionOnce, TestStepErrorCorrectionOnce)
{
    const TestInputStepErrorCorrection& input = GetParam();

    const Angle initial_orientation = Angle::zero();
    const Angle final_orientation = Angle::quarter();
    const AngularVelocity initial_angular_velocity = AngularVelocity{Angle::zero()};
    const AngularVelocity maximum_angular_velocity = AngularVelocity{pi_over_4};
    const AngularAcceleration maximum_angular_acceleration = AngularVelocity{Angle::quarter() / 4.0};
    const AngularAcceleration maximum_angular_deceleration = AngularVelocity{Angle::quarter() / 4.0};
    const BangBangTrajectory1DAngular trajectory{initial_orientation, final_orientation, initial_angular_velocity, maximum_angular_velocity, maximum_angular_acceleration, maximum_angular_deceleration};
    const Duration elapsed_time = Duration::fromSeconds(2.0);
    const Duration delta_time = Duration::fromSeconds(1.0);

    const AngularVelocity actual = OrientationController().step(input.orienation, trajectory, elapsed_time, delta_time);

    EXPECT_EQ(input.expected, actual);
}

INSTANTIATE_TEST_SUITE_P(TestStepErrorCorrectionOnce, TestStepErrorCorrectionOnce,
    testing::Values(
        /** @brief Test for no error correction. */
        TestInputStepErrorCorrection
        {
            .expected=AngularVelocity{pi_over_4},
            .orienation=(pi_over_4)
        },
        /** @brief Test when orientation is short of expected orientation. */
        TestInputStepErrorCorrection
        {
            .expected=AngularVelocity{pi_over_4 + pi_over_4 / 2.0 * proportional},
            .orienation=(pi_over_4 - pi_over_4 / 2.0)
        },
        /** @brief Test when orientation is short of expected orientation. */
        TestInputStepErrorCorrection
        {
            .expected=AngularVelocity{pi_over_4 - pi_over_4 / 2.0 * proportional},
            .orienation=(pi_over_4 + pi_over_4 / 2.0)
        }
    )
);
