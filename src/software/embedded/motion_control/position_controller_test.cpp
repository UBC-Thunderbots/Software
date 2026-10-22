#include "software/embedded/motion_control/position_controller.h"

#include <gtest/gtest.h>

#include <memory>
#include "gtest/gtest.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_2d.h"
#include "software/ai/navigator/trajectory/kinematic_constraints.h"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/embedded/motion_control/pid_controller.h"
#include "software/geom/point.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"

// TODO(#3743): write proper tests after pid constants have been tuned

namespace {
    /** @note Meant to be private. */
    const PositionController _positionController;

    const PidController<double> xPid = _positionController.getXPid();
    const PidController<double> yPid = _positionController.getXPid();
}

/** @brief Construct a TrajectoryPath object.
 * @details Passes values directly to a constructor of BangBangTrajectory2D.
 * @note @ref constraints is intitialized with maximum velocity, acceleration,
 * and decceleration values of 10.
 * !!! velocity should not be nonzero
 */
TrajectoryPath create_trajectory_path(const Point& initial_pos, const Point& final_pos, const Vector& initial_vel);

struct TestStepInputTrivial 
{
public:
    const Vector expected;
    const Point position;
    const TrajectoryPath trajectory;
};

class TestStepTrivial : public testing::TestWithParam<TestStepInputTrivial>
{
};

/** @brief Parametrized test format when final point is reached. */
TEST_P(TestStepTrivial, TestStepTrivial) 
{
    const TestStepInputTrivial& input = GetParam();

    const Duration time = Duration::fromSeconds(1.0);

    const Vector actual = PositionController().step(input.position, input.trajectory, time, time);

    EXPECT_EQ(input.expected, actual);
}

INSTANTIATE_TEST_SUITE_P(TestStepTrivial, TestStepTrivial,
    testing::Values(
        /** @brief Test for the origin. */
        TestStepInputTrivial
        {
            .expected=Vector{},
            .position=Point{},
            .trajectory=TrajectoryPath{std::make_shared<BangBangTrajectory2D>(), BangBangTrajectory2D::generator},
        },
        /** @brief Test for a nonorigin point. */
        TestStepInputTrivial
        {
            .expected=Vector{},
            .position=Point{1.0, 1.0},
            .trajectory=create_trajectory_path
            (
                Point{1.0, 1.0}, 
                Point{1.0, 1.0}, 
                Vector{}
            )
        },
        /** @brief Test for a nonzero initial vector. */
        TestStepInputTrivial
        {
            .expected=Vector{},
            .position=Point{1.0, 1.0},
            .trajectory=create_trajectory_path
            (
                Point{},
                Point{1.0, 1.0},
                Vector{1.0, 1.0}
            )
        }
    )
);

/** @brief Test if robot can move in all directions. */
INSTANTIATE_TEST_SUITE_P(TestStepGenericMovement, TestStepTrivial, 
    testing::Values(
        /** @brief Test if robot can move positively along y = x. */
        TestStepInputTrivial
        {
            .expected=Vector{xPid.getProportionalGain(), yPid.getProportionalGain()}, // !!!
            .position=Point{2.0, 2.0},
            .trajectory=create_trajectory_path
            (
                Point{2.0, 2.0},
                Point{3.0, 3.0},
                Vector{}
            )
        },
        /** @brief Test if robot can move negatively along y = x. */
        TestStepInputTrivial
        {
            .expected=Vector{-xPid.getProportionalGain(), -yPid.getProportionalGain()},
            .position=Point{2.0, 2.0},
            .trajectory=create_trajectory_path
            (
                Point{2.0, 2.0}, 
                Point{1.0, 1.0}, 
                Vector{}
            )
        },
        /** @brief Test if robot can move leftward along y = -x. */
        TestStepInputTrivial
        {
            .expected=Vector{-xPid.getProportionalGain(), yPid.getProportionalGain()},
            .position=Point{2.0, 2.0},
            .trajectory=create_trajectory_path
            (
                Point{2.0, 2.0}, 
                Point{1.0, 3.0}, 
                Vector{}
            )
        },
        /** @brief Test if robot can move rightward along y = -x. */
        TestStepInputTrivial
        {
            .expected=Vector{xPid.getProportionalGain(), -yPid.getProportionalGain()},
            .position=Point{2.0, 2.0},
            .trajectory=create_trajectory_path
            (
                Point{2.0, 2.0}, 
                Point{3.0, 1.0}, 
                Vector{}
            )
        }
    )
);

struct TestStepInputErrorCorrection 
{
public:
    const Vector expected;
    const Point position;
    const TrajectoryPath trajectory;
    const Duration elapsed_time;
    const Duration delta_time;
};

class TestStepErrorCorrectionOnce : public testing::TestWithParam<TestStepInputErrorCorrection>
{
};

/** @brief Parametrized test format for testing error correction once. */
TEST_P(TestStepErrorCorrectionOnce, TestStepErrorCorrectionOnce) 
{
    const TestStepInputErrorCorrection& input = GetParam();

    const Vector actual = PositionController().step(input.position, input.trajectory, input.elapsed_time, input.delta_time);

    EXPECT_EQ(input.expected, actual);
}

// when stuck at some point
// when no error, so correct pos !!!
// correct again for some arbitrary value !!!
INSTANTIATE_TEST_SUITE_P(TestErrorCorrectionOnce, TestStepErrorCorrectionOnce,
    testing::Values(
        /** @brief !!! */
        TestStepInputErrorCorrection
        {
            Vector{0.5, 0.5}, // 0.1, 0.1
            Point{2.5, 2.5},
            create_trajectory_path(
                Point{2.0, 2.0}, 
                Point{3.0, 3.0}, 
                Vector{1.0, 1.0}
            ),
            Duration::fromSeconds(1.0),
            Duration::fromSeconds(1.0)
        }
    )
);

// !!! test continuous movement error correction, where each error amount is wihtin a margin?

TrajectoryPath create_trajectory_path(const Point& initial_pos, const Point& final_pos, const Vector& initial_vel) 
{
    constexpr double max_acceleration = 2.0;
    constexpr double max_deceleration = 2.0;
    constexpr double max_velocity = 5.0;

    const KinematicConstraints kinematic_constraints{max_velocity, max_acceleration, max_deceleration};

    return TrajectoryPath{std::make_shared<BangBangTrajectory2D>(BangBangTrajectory2D{initial_pos, final_pos, initial_vel, kinematic_constraints}), BangBangTrajectory2D::generator};
}

