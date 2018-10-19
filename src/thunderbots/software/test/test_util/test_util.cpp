#include "test/test_util/test_util.h"

namespace Test
{
    Field TestUtil::createSSLDivBField()
    {
        // Using the dimensions of a standard Division B SSL field
        Field field = Field(9.0, 6.0, 1.0, 2.0, 1.0, 0.3, 0.5);
        return field;
    }

    World TestUtil::createBlankTestingWorld()
    {
        Field field        = createSSLDivBField();
        Team friendly_team = Team(std::chrono::milliseconds(1000));
        Team enemy_team    = Team(std::chrono::milliseconds(1000));
        Ball ball          = Ball(Point(), Vector());

        World world = World(field, ball, friendly_team, enemy_team);

        return world;
    }

    Team TestUtil::setRobotPositionsHelper(Team team,
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

    World TestUtil::setFriendlyRobotPositions(World world,
                                              std::vector<Point> robot_positions)
    {
        Team new_friendly_team =
            setRobotPositionsHelper(world.friendlyTeam(), robot_positions);
        world.mutableFriendlyTeam().clearAllRobots();
        world.updateFriendlyTeamState(new_friendly_team);

        return world;
    }

    World TestUtil::setEnemyRobotPositions(World world,
                                           std::vector<Point> robot_positions)
    {
        Team new_enemy_team = setRobotPositionsHelper(world.enemyTeam(), robot_positions);
        world.mutableEnemyTeam().clearAllRobots();
        world.updateEnemyTeamState(new_enemy_team);

        return world;
    }

    World TestUtil::setBallPosition(World world, Point ball_position)
    {
        Ball ball = Ball(ball_position, world.ball().velocity(),
                         std::chrono::steady_clock::now());
        world.updateBallState(ball);

        return world;
    }

    World TestUtil::setBallVelocity(World world, Vector ball_velocity)
    {
        Ball ball = Ball(world.ball().position(), ball_velocity,
                         std::chrono::steady_clock::now());
        world.updateBallState(ball);

        return world;
    }
}  // namespace Test
