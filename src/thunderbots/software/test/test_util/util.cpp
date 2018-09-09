#include "util.h"

namespace UnitTest
{
    Field Util::createNormalTestingField()
    {
        Field field = Field();
        // Using the dimensions of a standard Division B SSL field
        field.updateDimensions(9.0, 6.0, 1.0, 1.0, 2.0, 0.3, 0.5);

        return field;
    }

    World Util::createNormalTestingWorld()
    {
        Field field        = createNormalTestingField();
        Team friendly_team = Team();
        Team enemy_team    = Team();
        Ball ball          = Ball();

        World world = World(field, ball, friendly_team, enemy_team);

        return world;
    }

    Team Util::setRobotPositionsHelper(Team team,
                                       const std::vector<Point>& robot_positions)
    {
        std::vector<Robot> robots;
        unsigned int robot_index = 0;
        for (const Point& robot_position : robot_positions)
        {
            Robot robot = Robot(robot_index);
            robot.update(robot_position, Vector(), Angle::zero(),
                         AngularVelocity::zero());
            robots.emplace_back(robot);

            robot_index++;
        }

        team.clearAllRobots();
        team.updateRobots(robots);

        return team;
    }

    World Util::setFriendlyRobotPositions(World world, std::vector<Point> robot_positions)
    {
        Team new_friendly_team =
            setRobotPositionsHelper(world.friendly_team(), robot_positions);
        world.clearFriendlyTeamRobots();
        world.updateFriendlyTeam(new_friendly_team);

        return world;
    }

    World Util::setEnemyRobotPositions(World world, std::vector<Point> robot_positions)
    {
        Team new_enemy_team =
            setRobotPositionsHelper(world.enemy_team(), robot_positions);
        world.clearEnemyTeamRobots();
        world.updateEnemyTeam(new_enemy_team);

        return world;
    }

    World Util::setBallPosition(World world, Point ball_position)
    {
        Ball ball = world.ball();
        ball.update(ball_position, world.ball().velocity());
        world.updateBallState(ball);

        return world;
    }

    World Util::setBallVelocity(World world, Vector ball_velocity)
    {
        Ball ball = world.ball();
        ball.update(world.ball().position(), ball_velocity);
        world.updateBallState(ball);

        return world;
    }
}