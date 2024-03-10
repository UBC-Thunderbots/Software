#pragma once

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>

#include "shared/constants.h"
#include "software/geom/algorithms/almost_equal.h"
#include "software/geom/geom_constants.h"
#include "software/geom/point.h"
#include "software/geom/polygon.h"
#include "software/geom/stadium.h"
#include "software/physics/physics.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/team.h"
#include "software/world/world.h"

namespace TestUtil
{
    /**
     * Checks if two durations are within tolerance of each other
     * Two durations are within tolerance of each other if the absolute duration
     * difference is less than the tolerance duration
     *
     * @param duration1, duration2 Durations to compare
     * @param tolerance tolerance to check equality with, default is 1 microsecond (1/1000
     * millisecond)
     *
     * @return AssertionSuccess if the two durations are within tolerance of each other
     */
    ::testing::AssertionResult equalWithinTolerance(
        const Duration &duration1, const Duration &duration2,
        const Duration &tolerance = Duration::fromMilliseconds(0.001));

    /**
     * Checks if two polygons are within tolerance of each other
     * Two polygons are within tolerance of each other if the corresponding points are
     * within tolerance of each other
     *
     * @param poly1, poly2 Polygons to compare
     * @param tolerance tolerance to check equality with, default is
     * METERS_PER_MILLIMETER
     *
     * @return AssertionSuccess if the two polygons are within tolerance of each other
     */
    ::testing::AssertionResult equalWithinTolerance(
        const Polygon &poly1, const Polygon &poly2,
        double tolerance = METERS_PER_MILLIMETER);

    /**
     * Checks if two stadiums are within tolerance of each other
     * Two stadiums are within tolerance of each other if the corresponding points are
     * within tolerance of each other and the radii are within tolerance
     *
     * @param stadium1, stadium2 Stadiums to compare
     * @param tolerance tolerance to check equality with, default is
     * METERS_PER_MILLIMETER
     *
     * @return AssertionSuccess if the two polygons are within tolerance of each other
     */
    ::testing::AssertionResult equalWithinTolerance(
        const Stadium &stadium1, const Stadium &stadium2,
        double tolerance = METERS_PER_MILLIMETER);

    /**
     * Checks if two circles are within tolerance of each other
     * Two circles are within tolerance of each other if the origins are within
     * tolerance of each other and radius is within tolerance of each other
     *
     * @param c1, c2 Circles to compare
     * @param tolerance tolerance to check equality with, default is
     * METERS_PER_MILLIMETER
     *
     * @return AssertionSuccess if the two circles are within tolerance of each other
     */
    ::testing::AssertionResult equalWithinTolerance(
        const Circle &c1, const Circle &c2, double tolerance = METERS_PER_MILLIMETER);

    /**
     * Checks if two Angles are within tolerance of each other
     *
     * @param a1, a2 Angles to compare
     * @param tolerance tolerance to check equality with
     *
     * @return AssertionSuccess if the two Angles are within tolerance of each other
     */
    ::testing::AssertionResult equalWithinTolerance(const Angle &a1, const Angle &a2,
                                                    const Angle &tolerance);

    /**
     * Checks if two vectors are within tolerance of each other
     * Two vectors are within tolerance of each other if the respective x and y values
     * are within tolerance of each other
     *
     * @param v1, v2 Vectors to compare
     * @param tolerance tolerance to check equality with, default is
     * METERS_PER_MILLIMETER
     *
     * @return AssertionSuccess if the two points are within tolerance of each other
     */
    ::testing::AssertionResult equalWithinTolerance(const Vector &v1, const Vector &v2,
                                                    double tolerance);

    /**
     * Checks if two points are within tolerance of each other
     * Two points are within tolerance of each other if the respective x and y values
     * are within tolerance of each other
     *
     * @param pt1, pt2 Points to compare
     * @param tolerance tolerance to check equality with, default is
     * METERS_PER_MILLIMETER
     *
     * @return AssertionSuccess if the two points are within tolerance of each other
     */
    ::testing::AssertionResult equalWithinTolerance(const Point &pt1, const Point &pt2,
                                                    double tolerance);

    /**
     * Checks if two values are within tolerance of each other
     *
     * @param val1, val2 values to compare
     * @param tolerance tolerance to check equality with, default is
     * METERS_PER_MILLIMETER
     *
     * @return AssertionSuccess if the two values are within tolerance of each other
     */
    ::testing::AssertionResult equalWithinTolerance(double val1, double val2,
                                                    double tolerance);

    /**
     * Checks if two RobotStates are within tolerance of each other.
     * Two states are within tolerance if each other if their positions and velocities
     * are within tolerance of each other
     *
     * @param state1, state2 states to compare
     * @param tolerance The tolerance to check equality with
     *
     * @return AssertionSuccess if the two values are within tolerance of each other
     */
    ::testing::AssertionResult equalWithinTolerance(const RobotState &state1,
                                                    const RobotState &state2,
                                                    double linear_tolerance,
                                                    const Angle &angular_tolerance);

    /**
     * Checks if two RobotStateWithIds are within tolerance of each other.
     * Two states are within tolerance if their states are within tolerance
     * of each other
     *
     * @param state1, state2 states to compare
     * @param linear_tolerance The tolerance to check positions and velocities with
     * @param angular_tolerance The tolerance to check orientations and angular
     * velocities with
     *
     * @return AssertionSuccess if the two values are within tolerance of each other
     */
    ::testing::AssertionResult equalWithinTolerance(const RobotStateWithId &state1,
                                                    const RobotStateWithId &state2,
                                                    double linear_tolerance,
                                                    const Angle &angular_tolerance);

    /**
     * Checks if two BallStates are within tolerance of each other.
     * Two states are within tolerance if their states are within tolerance
     * of each other
     *
     * @param state1, state2 states to compare
     * @param linear_tolerance The tolerance to check positions and velocities with
     * @param angular_tolerance The tolerance to check orientations and angular
     * velocities with
     *
     * @return AssertionSuccess if the two values are within tolerance of each other
     */
    ::testing::AssertionResult equalWithinTolerance(const BallState &state1,
                                                    const BallState &state2,
                                                    double tolerance);

    /**
     * Checks if two matrices are within tolerance of each other.
     * Two matrices are within tolerance if their norms are within tolerance of each
     * other.
     *
     * @param matrix1, matrix2 The matrices to compare.
     * @param tolerance The tolerance to check velocities with.
     * @return AssertionSuccess if the two states are within tolerance of each other.
     */
    ::testing::AssertionResult equalWithinTolerance(const Eigen::MatrixXd &matrix1,
                                                    const Eigen::MatrixXd &matrix2,
                                                    double tolerance);
};  // namespace TestUtil
