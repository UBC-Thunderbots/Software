#include "software/test_util/test_util.h"

#include "shared/proto/robot_log_msg.nanopb.h"
#include "shared/proto/robot_log_msg.pb.h"
#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"

namespace TestUtil
{
    World createBlankTestingWorld()
    {
        Field field        = Field::createSSLDivisionBField();
        Team friendly_team = Team(Duration::fromMilliseconds(1000));
        Team enemy_team    = Team(Duration::fromMilliseconds(1000));
        Ball ball          = Ball(Point(), Vector(), Timestamp::fromSeconds(0));

        World world = World(field, ball, friendly_team, enemy_team);

        return world;
    }

    Team setRobotPositionsHelper(Team team, const std::vector<Point> &robot_positions,
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

    World setFriendlyRobotPositions(World world, std::vector<Point> robot_positions,
                                    const Timestamp &timestamp)
    {
        Team new_friendly_team =
            setRobotPositionsHelper(world.friendlyTeam(), robot_positions, timestamp);
        world.updateFriendlyTeamState(new_friendly_team);

        return world;
    }

    World setEnemyRobotPositions(World world, std::vector<Point> robot_positions,
                                 const Timestamp &timestamp)
    {
        Team new_enemy_team =
            setRobotPositionsHelper(world.enemyTeam(), robot_positions, timestamp);
        world.updateEnemyTeamState(new_enemy_team);

        return world;
    }

    World setBallPosition(World world, Point ball_position, Timestamp timestamp)
    {
        BallState ball_state(ball_position, world.ball().velocity());
        world.updateBall(Ball(ball_state, timestamp));

        return world;
    }

    World setBallVelocity(World world, Vector ball_velocity, Timestamp timestamp)
    {
        BallState ball_state(world.ball().position(), ball_velocity);
        world.updateBall(Ball(ball_state, timestamp));

        return world;
    }

    Robot createRobotAtPos(const Point &pt)
    {
        static RobotId robot_id_counter = 0;
        return Robot(robot_id_counter++, pt, Vector(), Angle(), AngularVelocity(),
                     Timestamp());
    }

    double secondsSince(std::chrono::time_point<std::chrono::system_clock> start_time)
    {
        return millisecondsSince(start_time) / MILLISECONDS_PER_SECOND;
    }

    double millisecondsSince(
        std::chrono::time_point<std::chrono::system_clock> start_time)
    {
        const auto end_time = std::chrono::system_clock::now();
        return static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       end_time - start_time)
                                       .count()) /
               NANOSECONDS_PER_MILLISECOND;
    }

    std::vector<RobotStateWithId> createStationaryRobotStatesWithId(
        const std::vector<Point> &positions)
    {
        std::vector<RobotStateWithId> states;
        for (RobotId id = 0; id < static_cast<RobotId>(positions.size()); id++)
        {
            states.push_back(RobotStateWithId{
                .id          = id,
                .robot_state = RobotState(positions[id], Vector(0, 0), Angle::zero(),
                                          AngularVelocity::zero())});
        }
        return states;
    }

    void handleTestRobotLog(TbotsProto_RobotLog robot_log)
    {
        LOG(INFO) << "[TEST ROBOT " << robot_log.robot_id << "]["
                  << TbotsProto::LogLevel_Name(
                         static_cast<TbotsProto::LogLevel>(robot_log.log_level))
                  << "]"
                  << "[" << robot_log.file_name << ":" << robot_log.line_number
                  << "]: " << robot_log.log_msg;
    }

};  // namespace TestUtil
