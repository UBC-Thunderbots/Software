#include "util.h"

namespace UnitTest
{
    Field Util::createSSLDivBField()
    {
        Field field = Field();
        // Using the dimensions of a standard Division B SSL field
        field.updateDimensions(9.0, 6.0, 1.0, 2.0, 1.0, 0.3, 0.5);

        return field;
    }

    World Util::createBlankTestingWorld()
    {
        Field field        = createSSLDivBField();
        Team friendly_team = Team(std::chrono::milliseconds(1000));
        Team enemy_team    = Team(std::chrono::milliseconds(1000));
        Ball ball          = Ball();

        World world = World(field, ball, friendly_team, enemy_team);

        return world;
    }

    Team Util::setRobotPositionsHelper(Team team,
                                       const std::vector<Point> &robot_positions)
    {
        std::vector<Robot> robots;
        unsigned int robot_id_index = 0;
        for (const Point &robot_position : robot_positions)
        {
            Robot robot =
                Robot(robot_id_index, robot_position, Vector(), Angle::zero(),
                      AngularVelocity::zero(), std::chrono::steady_clock::now());
            robots.emplace_back(robot);

            robot_id_index++;
        }

        team.clearAllRobots();
        team.updateRobots(robots);

        return team;
    }

    World Util::setFriendlyRobotPositions(World world, std::vector<Point> robot_positions)
    {
        Team new_friendly_team =
            setRobotPositionsHelper(world.friendlyTeam(), robot_positions);
        world.mutableFriendlyTeam().clearAllRobots();
        world.updateFriendlyTeamState(new_friendly_team);

        return world;
    }

    World Util::setEnemyRobotPositions(World world, std::vector<Point> robot_positions)
    {
        Team new_enemy_team = setRobotPositionsHelper(world.enemyTeam(), robot_positions);
        world.mutableEnemyTeam().clearAllRobots();
        world.updateEnemyTeamState(new_enemy_team);

        return world;
    }

    World Util::setBallPosition(World world, Point ball_position)
    {
        Ball ball = Ball(ball_position, world.ball().velocity(),
                         std::chrono::steady_clock::now());
        world.updateBallState(ball);

        return world;
    }

    World Util::setBallVelocity(World world, Vector ball_velocity)
    {
        Ball ball = Ball(world.ball().position(), ball_velocity,
                         std::chrono::steady_clock::now());
        world.updateBallState(ball);

        return world;
    }
}  // namespace UnitTest