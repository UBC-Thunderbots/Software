#include "util/ros_messages.h"

namespace Util
{
    namespace ROSMessages
    {
        Ball createBallFromROSMessage(const thunderbots_msgs::Ball& ball_msg)
        {
            Point ball_position  = Point(ball_msg.position.x, ball_msg.position.y);
            Vector ball_velocity = Vector(ball_msg.velocity.x, ball_msg.velocity.y);
            Timestamp timestamp  = Timestamp::fromSeconds(ball_msg.timestamp_seconds);

            Ball ball = Ball(ball_position, ball_velocity, timestamp);

            return ball;
        }

        thunderbots_msgs::Ball convertBallToROSMessage(const Ball& ball)
        {
            thunderbots_msgs::Ball ball_msg;

            ball_msg.position.x = ball.position().x();
            ball_msg.position.y = ball.position().y();

            ball_msg.velocity.x = ball.velocity().x();
            ball_msg.velocity.y = ball.velocity().y();

            ball_msg.timestamp_seconds = ball.lastUpdateTimestamp().getSeconds();

            return ball_msg;
        }

        Robot createRobotFromROSMessage(const thunderbots_msgs::Robot& robot_msg)
        {
            unsigned int robot_id   = robot_msg.id;
            Point robot_position    = Point(robot_msg.position.x, robot_msg.position.y);
            Vector robot_velocity   = Vector(robot_msg.velocity.x, robot_msg.velocity.y);
            Angle robot_orientation = Angle::ofRadians(robot_msg.orientation);
            AngularVelocity robot_angular_velocity =
                Angle::ofRadians(robot_msg.angular_velocity);
            Timestamp timestamp = Timestamp::fromSeconds(robot_msg.timestamp_seconds);

            Robot robot = Robot(robot_id, robot_position, robot_velocity,
                                robot_orientation, robot_angular_velocity, timestamp);

            return robot;
        }

        thunderbots_msgs::Robot convertRobotToROSMessage(const Robot& robot)
        {
            thunderbots_msgs::Robot robot_msg;

            robot_msg.id = robot.id();

            robot_msg.position.x = robot.position().x();
            robot_msg.position.y = robot.position().y();

            robot_msg.velocity.x = robot.velocity().x();
            robot_msg.velocity.y = robot.velocity().y();

            robot_msg.orientation       = robot.orientation().toRadians();
            robot_msg.angular_velocity  = robot.angularVelocity().toRadians();
            robot_msg.timestamp_seconds = robot.lastUpdateTimestamp().getSeconds();

            return robot_msg;
        }

        Field createFieldFromROSMessage(const thunderbots_msgs::Field& field_msg)
        {
            Field field = Field(field_msg.field_length, field_msg.field_width,
                                field_msg.defense_length, field_msg.defense_width,
                                field_msg.goal_width, field_msg.boundary_width,
                                field_msg.center_circle_radius);

            return field;
        }

        thunderbots_msgs::Field convertFieldToROSMessage(const Field& field)
        {
            thunderbots_msgs::Field field_msg;

            field_msg.field_length         = field.length();
            field_msg.field_width          = field.width();
            field_msg.defense_length       = field.defenseAreaLength();
            field_msg.defense_width        = field.defenseAreaWidth();
            field_msg.goal_width           = field.goalWidth();
            field_msg.boundary_width       = field.boundaryWidth();
            field_msg.center_circle_radius = field.centreCircleRadius();

            return field_msg;
        }

        Team createTeamFromROSMessage(const thunderbots_msgs::Team& team_msg)
        {
            std::vector<Robot> robots;
            for (const auto& robot_msg : team_msg.robots)
            {
                Robot robot = createRobotFromROSMessage(robot_msg);
                robots.emplace_back(robot);
            }

            auto expiry_buffer =
                Duration::fromMilliseconds(team_msg.robot_expiry_buffer_milliseconds);

            Team team = Team(expiry_buffer);

            team.updateRobots(robots);

            if (team_msg.goalie_id >= 0)
            {
                // TODO: Remove this exception handling. It is a temporary fix for
                // https://github.com/UBC-Thunderbots/Software/issues/287
                // and should be fixed and removed as part of
                // https://github.com/UBC-Thunderbots/Software/issues/230
                try
                {
                    team.assignGoalie(static_cast<unsigned int>(team_msg.goalie_id));
                }
                catch (std::invalid_argument)
                {
                    // Do nothing
                }
            }
            else
            {
                team.clearGoalie();
            }

            return team;
        }

        thunderbots_msgs::Team convertTeamToROSMessage(const Team& team)
        {
            thunderbots_msgs::Team team_msg;

            team_msg.robot_expiry_buffer_milliseconds =
                team.getRobotExpiryBufferDuration().getMilliseconds();

            if (team.getGoalieID())
            {
                team_msg.goalie_id = *team.getGoalieID();
            }
            else
            {
                team_msg.goalie_id = -1;
            }

            auto robots     = team.getAllRobots();
            auto robot_msgs = std::vector<thunderbots_msgs::Robot>();
            for (auto robot : robots)
            {
                robot_msgs.emplace_back(convertRobotToROSMessage(robot));
            }
            team_msg.robots = robot_msgs;

            return team_msg;
        }

        RefboxGameState createGameStateFromROSMessage(
            const thunderbots_msgs::RefboxCommand& command)
        {
            // the values in RefboxGameState correspond to the constants in
            // RefboxCommand.msg
            return (RefboxGameState)command.command;
        }

        World createWorldFromROSMessage(const thunderbots_msgs::World& world_msg)
        {
            Ball ball          = createBallFromROSMessage(world_msg.ball);
            Team friendly_team = createTeamFromROSMessage(world_msg.friendly_team);
            Team enemy_team    = createTeamFromROSMessage(world_msg.enemy_team);
            Field field        = createFieldFromROSMessage(world_msg.field);

            // TODO: ???
            //            RefboxGamState refbox_gamestate =
            //            createGameStateFromROSMessage(world_msg.refbox_data);

            World world(field, ball, friendly_team, enemy_team);
            return world;
        }

        thunderbots_msgs::World convertWorldToROSMessage(const World& world)
        {
            thunderbots_msgs::World world_msg;

            world_msg.field         = convertFieldToROSMessage(world.field());
            world_msg.ball          = convertBallToROSMessage(world.ball());
            world_msg.friendly_team = convertTeamToROSMessage(world.friendlyTeam());
            world_msg.enemy_team    = convertTeamToROSMessage(world.enemyTeam());
            // world_msg.refbox_data = world.refBoxData();
            // TODO add function "refBoxData()"

            return world_msg;
        }

        thunderbots_msgs::World invertMsgFieldSide(
            const thunderbots_msgs::World& old_world_msg)
        {
            thunderbots_msgs::World new_world_msg = old_world_msg;

            new_world_msg.ball = invertMsgFieldSide(old_world_msg.ball);
            new_world_msg.friendly_team.robots =
                invertMsgFieldSide(old_world_msg.friendly_team.robots);
            new_world_msg.enemy_team.robots =
                invertMsgFieldSide(old_world_msg.enemy_team.robots);

            return new_world_msg;
        }

        thunderbots_msgs::Ball invertMsgFieldSide(
            const thunderbots_msgs::Ball& old_ball_msg)
        {
            thunderbots_msgs::Ball new_ball = old_ball_msg;

            new_ball.position.x = -old_ball_msg.position.x;
            new_ball.position.y = -old_ball_msg.position.y;

            new_ball.velocity.x = -old_ball_msg.velocity.x;
            new_ball.velocity.y = -old_ball_msg.velocity.y;

            return new_ball;
        }

        std::vector<thunderbots_msgs::Robot> invertMsgFieldSide(
            const std::vector<thunderbots_msgs::Robot>& old_robot_msgs)
        {
            std::vector<thunderbots_msgs::Robot> new_robots;

            for (auto old_robot : old_robot_msgs)
            {
                new_robots.emplace_back(invertMsgFieldSide(old_robot));
            }

            return new_robots;
        }

        thunderbots_msgs::Robot invertMsgFieldSide(
            const thunderbots_msgs::Robot& old_robot_msg)
        {
            thunderbots_msgs::Robot new_robot = old_robot_msg;

            new_robot.position.x = -old_robot_msg.position.x;
            new_robot.position.y = -old_robot_msg.position.y;

            new_robot.velocity.x = -old_robot_msg.velocity.x;
            new_robot.velocity.y = -old_robot_msg.velocity.y;

            new_robot.orientation = old_robot_msg.orientation + Angle::half().toRadians();

            return new_robot;
        }
    }  // namespace ROSMessages
}  // namespace Util
