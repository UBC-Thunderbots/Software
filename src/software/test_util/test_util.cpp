#include "software/test_util/test_util.h"

namespace Test
{
    Field TestUtil::createSSLDivBField()
    {
        // Using the dimensions of a standard Division B SSL field
        Field field = Field(9.0, 6.0, 1.0, 2.0, 1.0, 0.3, 0.5, Timestamp::fromSeconds(0));
        return field;
    }

    World TestUtil::createBlankTestingWorld()
    {
        Field field        = createSSLDivBField();
        Team friendly_team = Team(Duration::fromMilliseconds(1000));
        Team enemy_team    = Team(Duration::fromMilliseconds(1000));
        Ball ball          = Ball(Point(), Vector(), Timestamp::fromSeconds(0));

        World world = World(field, ball, friendly_team, enemy_team);

        return world;
    }

    Team TestUtil::setRobotPositionsHelper(Team team,
                                           const std::vector<Point> &robot_positions,
                                           const Timestamp &timestamp)
    {
        std::vector<Robot> robots;
        unsigned int robot_id_index = 0;
        for (const Point &robot_position : robot_positions)
        {
            Robot robot = Robot(robot_id_index, robot_position, Vector(), Angle::zero(),
                                AngularVelocity::zero(), timestamp);
            robots.emplace_back(robot);

            robot_id_index++;
        }

        team.clearAllRobots();
        team.updateRobots(robots);

        return team;
    }

    World TestUtil::setFriendlyRobotPositions(World world,
                                              std::vector<Point> robot_positions,
                                              const Timestamp &timestamp)
    {
        Team new_friendly_team =
            setRobotPositionsHelper(world.friendlyTeam(), robot_positions, timestamp);
        world.mutableFriendlyTeam().clearAllRobots();
        world.updateFriendlyTeamState(new_friendly_team);

        return world;
    }

    World TestUtil::setEnemyRobotPositions(World world,
                                           std::vector<Point> robot_positions,
                                           const Timestamp &timestamp)
    {
        Team new_enemy_team =
            setRobotPositionsHelper(world.enemyTeam(), robot_positions, timestamp);
        world.mutableEnemyTeam().clearAllRobots();
        world.updateEnemyTeamState(new_enemy_team);

        return world;
    }

    World TestUtil::setBallPosition(World world, Point ball_position, Timestamp timestamp)
    {
        TimestampedBallState ballState =
            TimestampedBallState(ball_position, world.ball().velocity(), timestamp);
        world.updateBallStateWithTimestamp(ballState);

        return world;
    }

    World TestUtil::setBallVelocity(World world, Vector ball_velocity,
                                    Timestamp timestamp)
    {
        TimestampedBallState ballState =
            TimestampedBallState(world.ball().position(), ball_velocity, timestamp);
        world.updateBallStateWithTimestamp(ballState);

        return world;
    }

    std::vector<RefboxGameState> TestUtil::getAllRefboxGameStates()
    {
        std::vector<RefboxGameState> game_states;
        for (int i = 0; i < static_cast<int>(RefboxGameState::REFBOX_GAME_STATE_COUNT);
             i++)
        {
            game_states.push_back(static_cast<RefboxGameState>(i));
        }
        return game_states;
    }

    Robot TestUtil::createRobotAtPos(const Point &pt)
    {
        static RobotId robot_id_counter = 0;
        return Robot(robot_id_counter++, pt, Vector(), Angle(), AngularVelocity(),
                     Timestamp());
    }

    bool TestUtil::equalWithinTolerance(const Polygon &poly1, const Polygon &poly2,
                                        double tolerance)
    {
        auto ppts1 = poly1.getPoints();
        auto ppts2 = poly2.getPoints();
        return std::equal(ppts1.begin(), ppts1.end(), ppts2.begin(),
                          [tolerance](const Point &p1, const Point &p2) {
                              return equalWithinTolerance(p1, p2, tolerance);
                          });
    }

    bool TestUtil::equalWithinTolerance(const Circle &c1, const Circle &c2,
                                        double tolerance)
    {
        return equalWithinTolerance(c1.getOrigin(), c2.getOrigin(), tolerance) &&
               equalWithinTolerance(c1.getRadius(), c2.getRadius(), tolerance);
    }

    bool TestUtil::equalWithinTolerance(const Point &pt1, const Point &pt2,
                                        double tolerance)
    {
        double distance = pt1.distanceFromPoint(pt2);
        return equalWithinTolerance(distance, 0, tolerance);
    }

    bool TestUtil::equalWithinTolerance(double val1, double val2, double tolerance)
    {
        // subtracting one fixed epsilon to account for the error in fabs and one fixed
        // epsilon to account for the error in subtracting the two vals
        double difference = fabs(val1 - val2) - GeomConstants::FIXED_EPSILON * 2;
        return difference < tolerance;
    }
}  // namespace Test
