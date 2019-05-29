#pragma once

#include "ai/world/ball.h"
#include "ai/world/field.h"
#include "ai/world/team.h"
#include "ai/world/world.h"

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
