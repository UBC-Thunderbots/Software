#pragma

#include "ai/world/ball.h"
#include "ai/world/field.h"
#include "ai/world/team.h"
#include "ai/world/world.h"

namespace UnitTest
{
    /**
     * This util class is to provide utility functions to our unit test suite, primarily
     * for
     * assisting with test setup (such as creating World or Field objects)
     */
    class Util
    {
       public:
        /**
         * Creates a field with the standard SSL Division B dimensions
         * @return a field with the standard SSL Division B dimensions
         */
        static Field createNormalTestingField();

        /**
         * Creates a World object with a normal testing field, and the Teams and Ball
         * initialized to default values so they are ready to be set for testing
         * @return a World object initialized with default values
         */
        static World createNormalTestingWorld();

        /**
         * Returns a new World object with friendly robots in the positions specified in
         * the
         * vector of Points. Robots will be created with consecutive id's from 0 to
         * robot_positions.size() - 1
         * @param world The current world object
         * @param robot_positions The positions to place friendly robots in
         * @return A new world object with friendly robots in the given positions
         */
        static World setFriendlyRobotPositions(World world,
                                               std::vector<Point> robot_positions);

        /**
         * Returns a new World object with enemy robots in the positions specified in the
         * vector of Points. Robots will be created with consecutive id's from 0 to
         * robot_positions.size() - 1
         * @param world The current world object
         * @param robot_positions The positions to place enemy robots in
         * @return A new world object with enemy robots in the given positions
         */
        static World setEnemyRobotPositions(World world,
                                            std::vector<Point> robot_positions);

        /**
         * Returns a new World object with the Ball placed in the new position specified
         * @param world The current world object
         * @param ball_position The new position for the ball
         * @return A new World object with the ball placed in the given position
         */
        static World setBallPosition(World world, Point ball_position);

        /**
         * Returns a new World object with the Ball's velocity set to the new velocity
         * @param world The current world object
         * @param ball_velocity The new velocity for the ball
         * @return A new World object with the ball's velocity set to the new velocity
         */
        static World setBallVelocity(World world, Vector ball_velocity);

       private:
        /**
         * Returns a new team with robots placed at the given positions. Robots in the
         * given team are removed before new ones are placed at the given positions, so
         * pre-existing robots to not persist.
         *
         * @param team The team for which to set robot positions
         * @param robot_positions The positions of the robots
         * @return A new team with robots placed at the given positions
         */
        static Team setRobotPositionsHelper(Team team,
                                            const std::vector<Point>& robot_positions);
    };
}
