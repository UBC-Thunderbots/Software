#include "software/embedded/motion_control/position_controller.h"

#include <gtest/gtest.h>

#include <memory>
#include <tuple>
#include "gtest/gtest.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_2d.h"
#include "software/ai/navigator/trajectory/kinematic_constraints.h"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/geom/point.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"

// TODO(#3743): write proper tests after pid constants have been tuned

class TestPositionControllerStep : public testing::TestWithParam<std::tuple<const Vector, const Point, const TrajectoryPath, const Duration, const Duration>>
{
};

TEST_P(TestPositionControllerStep, TestPositionControllerStep) 
{
    auto [expected, position, trajectory, elapsed_time, delta_time] = GetParam();
    PositionController controller;

    const Vector actual = controller.step(position, trajectory, elapsed_time, delta_time);

    EXPECT_EQ(expected, actual);

    controller.reset();
}

/** @brief Construct a TrajectoryPath object.
 * @details Passes values directly to a constructor of BangBangTrajectory2D.
 * @note @ref constraints is intitialized with maximum velocity, acceleration,
 * and decceleration values of 10.
 */
TrajectoryPath create_trajectory_path(const Point& initial_pos, const Point& final_pos, const Vector& initial_vel);

/** @brief Test when final point is reached. */
INSTANTIATE_TEST_SUITE_P(TestTrivial, TestPositionControllerStep,
    testing::Values(
        /** @brief Test for the origin. */
        std::tuple
        {
            Vector{},
            Point{},
            TrajectoryPath{std::make_shared<BangBangTrajectory2D>(), BangBangTrajectory2D::generator},
            Duration::fromSeconds(1.0),
            Duration::fromSeconds(1.0)
        },
        /** @brief Test for a nonorigin point. */
        std::tuple
        {
            Vector{},
            Point{1.0, 1.0},
            create_trajectory_path(
                Point{1.0, 1.0}, 
                Point{1.0, 1.0}, 
                Vector{}
            ),
            Duration::fromSeconds(1.0),
            Duration::fromSeconds(1.0)
        },
        /** @brief Test for a nonzero initial vector. */
        std::tuple
        {
            Vector{},
            Point{1.0, 1.0},
            create_trajectory_path(
                Point{},
                Point{1.0, 1.0},
                Vector{1.0, 1.0}
            ),
            Duration::fromSeconds(1.0),
            Duration::fromSeconds(1.0)
        }
    )
);

/** @brief Test movement when no initial vector is given. */
INSTANTIATE_TEST_SUITE_P(TestGenericMovement, TestPositionControllerStep,
    testing::Values(
        /** @brief Test positive movement along the x-axis. */
        std::tuple
        {
            Vector{1.0, 0.0, }, // !!!
            Point{2.0, 2.0},
            create_trajectory_path(
                Point{2.0, 2.0}, 
                Point{3.0, 2.0}, 
                Vector{}
            ),
            Duration::fromSeconds(1.0),
            Duration::fromSeconds(1.0)
        },
        /** @brief Test positive movement along the y-axis. */
        std::tuple
        {
            Vector{0.0, 1.0}, // !!!
            Point{2.0, 2.0},
            create_trajectory_path(
                Point{2.0, 2.0}, 
                Point{2.0, 3.0}, 
                Vector{}
            ),
            Duration::fromSeconds(1.0),
            Duration::fromSeconds(1.0)
        },
        /** @brief Test negative movement along the x-axis. */
        std::tuple
        {
            Vector{-1.0, 0.0}, // !!!
            Point{2.0, 2.0},
            create_trajectory_path(
                Point{3.0, 2.0}, 
                Point{2.0, 2.0}, 
                Vector{}
            ),
            Duration::fromSeconds(1.0),
            Duration::fromSeconds(1.0)
        },
        /** @brief Test negative movement along the y-axis. */
        std::tuple
        {
            Vector{0.0, 1.0}, // !!!
            Point{2.0, 2.0},
            create_trajectory_path(
                Point{2.0, 2.0}, 
                Point{2.0, 3.0}, 
                Vector{}
            ),
            Duration::fromSeconds(1.0),
            Duration::fromSeconds(1.0)
        }
    )
);

// INSTANTIATE_TEST_SUITE_P(TestErrorCorrection, TestPositionControllerStep,
//     testing::Values(
//         std::tuple{
//             // !!!
//         }
//     )
// );

TrajectoryPath create_trajectory_path(const Point& initial_pos, const Point& final_pos, const Vector& initial_vel) 
{
    constexpr double max_acceleration = 10.0;
    constexpr double max_deceleration = 10.0;
    constexpr double max_velocity = 10.0;

    const KinematicConstraints kinematic_constraints{max_velocity, max_acceleration, max_deceleration};

    return TrajectoryPath{std::make_shared<BangBangTrajectory2D>(BangBangTrajectory2D{initial_pos, final_pos, initial_vel, kinematic_constraints}), BangBangTrajectory2D::generator};
}
