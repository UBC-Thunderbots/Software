#pragma once

#include <gtest/gtest.h>

#include <chrono>

#include "shared/constants.h"
#include "shared/proto/robot_log_msg.nanopb.h"
#include "software/geom/algorithms/almost_equal.h"
#include "software/geom/geom_constants.h"
#include "software/geom/point.h"
#include "software/geom/polygon.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/team.h"
#include "software/world/world.h"

#define UNUSED(x) (void)(x)

/**
 * Utility functions to our unit test suite, primarily for assisting with test setup (such
 * as creating World or Field objects)
 */
namespace TestUtil
{
    /**
     * Creates a World object with a normal SSL Division B field, default (empty)
     * teams with 1000 milliseconds expiry buffers, and the Ball at the center of the
     * field with no velocity.
     *
     * @return a World object initialized with a Division B SSL field, empty teams
     * with 1000 millisecond expiry buffers, and the Ball at the center of the field
     * with no velocity.
     */
    World createBlankTestingWorld();

    /**
     * Returns a new World object with friendly robots in the positions specified
     * by the vector of Points. Robots will be created with consecutive increasing
     * id's from 0 to robot_positions.size() - 1. Any friendly robots that already
     * existed in the world are removed before the new ones are added.
     *
     * @param world The current world object
     * @param robot_positions The positions to place friendly robots in
     * @return A new world object with friendly robots in the given positions
     */
    World setFriendlyRobotPositions(World world, std::vector<Point> robot_positions,
                                    const Timestamp &timestamp);

    /**
     * Returns a new World object with enemy robots in the positions specified
     * by the vector of Points. Robots will be created with consecutive increasing
     * id's from 0 to robot_positions.size() - 1. Any enemy robots that already
     * existed in the world are removed before the new ones are added.
     *
     * @param world The current world object
     * @param robot_positions The positions to place enemy robots in
     * @return A new world object with enemy robots in the given positions
     */
    World setEnemyRobotPositions(World world, std::vector<Point> robot_positions,
                                 const Timestamp &timestamp);

    /**
     * Returns a new World object with the Ball placed in the new position
     * specified
     *
     * @param world The current world object
     * @param ball_position The new position for the ball
     * @return A new World object with the ball placed in the given position
     */
    World setBallPosition(World world, Point ball_position, Timestamp timestamp);

    /**
     * Returns a new World object with the Ball's velocity set to the new velocity
     *
     * @param world The current world object
     * @param ball_velocity The new velocity for the ball
     * @return A new World object with the ball's velocity set to the new velocity
     */
    World setBallVelocity(World world, Vector ball_velocity, Timestamp timestamp);

    /**
     * Returns a robot at the given position with zero velocity,
     * facing zero radians, with zero rad/s of angular velocity,
     * and an id monotonically increasing at each call of this
     * function
     * @param pt the point
     * @return a robot at the point
     */
    Robot createRobotAtPos(const Point &pt);

    /**
     * Gets the number of milliseconds since the start_time
     *
     * @param start_time time point to calculate time since
     *
     * @return milliseconds since start time
     */
    double millisecondsSince(
        std::chrono::time_point<std::chrono::system_clock> start_time);

    /**
     * Gets the number of seconds since the start_time
     *
     * @param start_time time point to calculate time since
     *
     * @return seconds since start time
     */
    double secondsSince(std::chrono::time_point<std::chrono::system_clock> start_time);

    /**
     * Returns a new team with robots placed at the given positions. Robots in the
     * given team are removed before new ones are placed at the given positions,
     * so pre-existing robots to not persist.
     *
     * @param team The team for which to set robot positions
     * @param robot_positions The positions of the robots
     * @return A new team with robots placed at the given positions
     */
    Team setRobotPositionsHelper(Team team, const std::vector<Point> &robot_positions,
                                 const Timestamp &timestamp);

    /**
     * Creates a list of RobotStateWithId at given positions with 0 velocity, 0 angular
     * velocity and 0 orientation The id is set as the index in the list of positions
     *
     * @param positions The positions to create robots at
     */
    std::vector<RobotStateWithId> createStationaryRobotStatesWithId(
        const std::vector<Point> &positions);
};  // namespace TestUtil
