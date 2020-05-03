#pragma once

#include "shared/constants.h"
#include "software/new_geom/geom_constants.h"
#include "software/new_geom/point.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/util/almost_equal.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/team.h"
#include "software/world/world.h"

namespace Test
{
    /**
     * This util class is to provide utility functions to our unit test suite,
     * primarily for assisting with test setup (such as creating World or Field objects)
     */
    class TestUtil
    {
       public:
        /**
         * Creates a field with the standard SSL Division B dimensions
         * @return a field with the standard SSL Division B dimensions
         */
        static Field createSSLDivBField();

        /**
         * Creates a World object with a normal SSL Division B field, default (empty)
         * teams with 1000 milliseconds expiry buffers, and the Ball at the center of the
         * field with no velocity.
         *
         * @return a World object initialized with a Division B SSL field, empty teams
         * with 1000 millisecond expiry buffers, and the Ball at the center of the field
         * with no velocity.
         */
        static World createBlankTestingWorld();

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
        static World setFriendlyRobotPositions(World world,
                                               std::vector<Point> robot_positions,
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
        static World setEnemyRobotPositions(World world,
                                            std::vector<Point> robot_positions,
                                            const Timestamp &timestamp);

        /**
         * Returns a new World object with the Ball placed in the new position
         * specified
         *
         * @param world The current world object
         * @param ball_position The new position for the ball
         * @return A new World object with the ball placed in the given position
         */
        static World setBallPosition(World world, Point ball_position,
                                     Timestamp timestamp);

        /**
         * Returns a new World object with the Ball's velocity set to the new velocity
         *
         * @param world The current world object
         * @param ball_velocity The new velocity for the ball
         * @return A new World object with the ball's velocity set to the new velocity
         */
        static World setBallVelocity(World world, Vector ball_velocity,
                                     Timestamp timestamp);

        /**
         * Returns a vector containing all Refbox game states except for
         * LAST_ENUM_ITEM_UNUSED
         *
         * @return A vector containing all Refbox game states except for
         * LAST_ENUM_ITEM_UNUSED
         */
        static std::vector<RefboxGameState> getAllRefboxGameStates();

        /**
         * Returns a robot at the given position with zero velocity,
         * facing zero radians, with zero rad/s of angular velocity,
         * and an id monotonically increasing at each call of this
         * function
         * @param pt the point
         * @return a robot at the point
         */
        static Robot createRobotAtPos(const Point &pt);

        /**
         * Checks if two polygons are within tolerance of each other
         * Two polygons are within tolerance of each other if the corresponding points are
         * within tolerance of each other
         *
         * @param poly1, poly2 Polygons to compare
         * @param tolerance tolerance to check equality with, default is
         * METERS_PER_MILLIMETER
         *
         * @return true if the two polygons are within tolerance of each other
         */
        static bool equalWithinTolerance(const Polygon &poly1, const Polygon &poly2,
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
         * @return true if the two circles are within tolerance of each other
         */
        static bool equalWithinTolerance(const Circle &c1, const Circle &c2,
                                         double tolerance = METERS_PER_MILLIMETER);

        /**
         * Checks if two points are within tolerance of each other
         * Two points are within tolerance of each other if the respective x and y values
         * are within tolerance of each other
         *
         * @param pt1, pt2 Points to compare
         * @param tolerance tolerance to check equality with, default is
         * METERS_PER_MILLIMETER
         *
         * @return true if the two points are within tolerance of each other
         */
        static bool equalWithinTolerance(const Point &pt1, const Point &pt2,
                                         double tolerance = METERS_PER_MILLIMETER);

        /**
         * Checks if two values are within tolerance of each other
         *
         * @param val1, val2 values to compare
         * @param tolerance tolerance to check equality with, default is
         * METERS_PER_MILLIMETER
         *
         * @return true if the two values are within tolerance of each other
         */
        static bool equalWithinTolerance(double val1, double val2,
                                         double tolerance = METERS_PER_MILLIMETER);

       private:
        /**
         * Returns a new team with robots placed at the given positions. Robots in the
         * given team are removed before new ones are placed at the given positions,
         * so pre-existing robots to not persist.
         *
         * @param team The team for which to set robot positions
         * @param robot_positions The positions of the robots
         * @return A new team with robots placed at the given positions
         */
        static Team setRobotPositionsHelper(Team team,
                                            const std::vector<Point> &robot_positions,
                                            const Timestamp &timestamp);
    };
}  // namespace Test
