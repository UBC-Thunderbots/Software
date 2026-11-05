#include "software/embedded/motion_control/position_controller.h"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "software/ai/navigator/trajectory/bang_bang_trajectory_2d.h"
#include "software/ai/navigator/trajectory/kinematic_constraints.h"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/embedded/motion_control/pid_controller.h"
#include "software/geom/point.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"

namespace
{
const PositionController _position_controller;
const PidController<double> _x_pid = _position_controller.getXPid();
const PidController<double> _y_pid = _position_controller.getXPid();

const double sqrt2          = std::sqrt(2.0);
const double x_proportional = _x_pid.getProportional();
const double x_derivative   = _x_pid.getDerivative();
const double x_integral     = _x_pid.getIntegral();
const double y_proportional = _y_pid.getProportional();
const double y_derivative   = _y_pid.getDerivative();
const double y_integral     = _y_pid.getIntegral();
}  // namespace

struct TestInputStepTrivial
{
   public:
    const Vector expected;
    const Point position;
    const TrajectoryPath trajectory;
};

class TestStepTrivial : public testing::TestWithParam<TestInputStepTrivial>
{
};

/** @brief Parametrized test for checking basic functionality. */
TEST_P(TestStepTrivial, TestStepTrivial)
{
    const TestInputStepTrivial& input = GetParam();

    const Duration time = Duration::fromSeconds(1.0);

    const Vector actual =
        PositionController().step(input.position, input.trajectory, time, time);

    EXPECT_EQ(input.expected, actual);
}

/** @brief Construct a TrajectoryPath object.
 * @details Passes values directly to a constructor of BangBangTrajectory2D.
 * @note Initializes with kinematic constraints of  maximum velocity, acceleration, and
 * decceleration values of 10.
 */
TrajectoryPath create_trajectory_path(const Point& initial_pos, const Point& final_pos);

/** @brief Sanity check. */
INSTANTIATE_TEST_SUITE_P(
    TestStepSanity, TestStepTrivial,
    testing::Values(
        /** @brief Test for the origin. */
        TestInputStepTrivial{
            .expected   = Vector{},
            .position   = Point{},
            .trajectory = TrajectoryPath{std::make_shared<BangBangTrajectory2D>(),
                                         BangBangTrajectory2D::generator},
        },
        /** @brief Test for a nonorigin point. */
        TestInputStepTrivial{
            .expected   = Vector{},
            .position   = Point{1.0, 1.0},
            .trajectory = create_trajectory_path(Point{}, Point{1.0, 1.0})}));

/** @brief Test if robot can move in all directions. */
INSTANTIATE_TEST_SUITE_P(
    TestStepGenericMovement, TestStepTrivial,
    testing::Values(
        /** @brief Test if robot can move positively along y = x. */
        TestInputStepTrivial{
            .expected   = Vector{x_proportional, y_proportional},
            .position   = Point{2.0, 2.0},
            .trajectory = create_trajectory_path(Point{2.0, 2.0}, Point{3.0, 3.0})},
        /** @brief Test if robot can move negatively along y = x. */
        TestInputStepTrivial{
            .expected   = Vector{-y_proportional, -y_proportional},
            .position   = Point{2.0, 2.0},
            .trajectory = create_trajectory_path(Point{2.0, 2.0}, Point{1.0, 1.0})},
        /** @brief Test if robot can move leftward along y = -x. */
        TestInputStepTrivial{
            .expected   = Vector{-x_proportional, y_proportional},
            .position   = Point{2.0, 2.0},
            .trajectory = create_trajectory_path(Point{2.0, 2.0}, Point{1.0, 3.0})},
        /** @brief Test if robot can move rightward along y = -x. */
        TestInputStepTrivial{
            .expected   = Vector{x_proportional, -y_proportional},
            .position   = Point{2.0, 2.0},
            .trajectory = create_trajectory_path(Point{2.0, 2.0}, Point{3.0, 1.0})}));

TrajectoryPath create_trajectory_path(const Point& initial_pos, const Point& final_pos)
{
    constexpr double max_acceleration = 10.0;
    constexpr double max_deceleration = 10.0;
    constexpr double max_velocity     = 10.0;

    const KinematicConstraints kinematic_constraints{max_velocity, max_acceleration,
                                                     max_deceleration};

    return TrajectoryPath{std::make_shared<BangBangTrajectory2D>(BangBangTrajectory2D{
                              initial_pos, final_pos, Vector{}, kinematic_constraints}),
                          BangBangTrajectory2D::generator};
}

struct TestInputStepErrorCorrection
{
   public:
    const Vector expected;
    const Point position;
};

class TestStepErrorCorrectionOnce
    : public testing::TestWithParam<TestInputStepErrorCorrection>
{
};

/** @brief Parametrized test for a single instance of error correction.
 * @details ils From 0uns^-1, we accelerate at 1uns^-2 for 2s to reach 1uns^-1,
 * plateau for (4sqrt2-2)s, and decelerate at 1uns^-2 for 2s to reach 0uns^1,
 * covering (8sqrt2)un of distance in (4qrt2+2)s.
 */
TEST_P(TestStepErrorCorrectionOnce, TestStepErrorCorrectionOnce)
{
    constexpr double max_acceleration = 1.0;
    constexpr double max_deceleration = 1.0;
    constexpr double max_velocity     = 2.0;

    const TestInputStepErrorCorrection& input = GetParam();

    const Point initial_position = Point{};
    const Point final_position   = Point{8.0, 8.0};
    const KinematicConstraints kinematic_constraints{max_velocity, max_acceleration,
                                                     max_deceleration};
    const TrajectoryPath trajectory{
        std::make_shared<BangBangTrajectory2D>(BangBangTrajectory2D{
            initial_position, final_position, Vector{}, kinematic_constraints}),
        BangBangTrajectory2D::generator};
    const Duration elapsed_time = Duration::fromSeconds(3.0);
    const Duration delta_time   = Duration::fromSeconds(1.0);

    const Vector actual =
        PositionController().step(input.position, trajectory, elapsed_time, delta_time);

    EXPECT_EQ(input.expected, actual);
}

INSTANTIATE_TEST_SUITE_P(
    TestErrorCorrectionOnce, TestStepErrorCorrectionOnce,
    testing::Values(
        /** @brief Test for no error correction. */
        TestInputStepErrorCorrection{
            .expected = Vector{sqrt2, sqrt2},
            .position = Point{4.0 / sqrt2, 4.0 / sqrt2},
        },
        /** @brief Test when actual position is short of expected position. */
        TestInputStepErrorCorrection{
            .expected = (Vector{sqrt2, sqrt2} +
                         Vector{x_proportional / sqrt2, y_proportional / sqrt2}),
            .position = Point{3.0 / sqrt2, 3.0 / sqrt2},
        },
        /** @brief Test when actual position is beyond expected position. */
        TestInputStepErrorCorrection{
            .expected = (Vector{sqrt2, sqrt2} -
                         Vector{x_proportional / sqrt2, y_proportional / sqrt2}),
            .position = Point{5.0 / sqrt2, 5.0 / sqrt2},
        },
        /** @brief Test when actual position is off to the side and above the expected
         * position.
         */
        TestInputStepErrorCorrection{
            .expected = (Vector{sqrt2, sqrt2} +
                         Vector{x_proportional / sqrt2, -y_proportional / sqrt2}),
            .position = Point{3.0 / sqrt2, 5.0 / sqrt2},
        },
        /** @brief Test when actual position is off to the side and below the expected
         * position.
         */
        TestInputStepErrorCorrection{
            .expected = (Vector{sqrt2, sqrt2} +
                         Vector{-x_proportional / sqrt2, y_proportional / sqrt2}),
            .position = Point{5.0 / sqrt2, 3.0 / sqrt2},
        }));

// !!! test integral

// !!! test derivative

// !!! test continuous movement error correction, where each error amount is within a
// margin?
