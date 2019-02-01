#include "util/ros_messages.h"

namespace Util
{
    namespace ROSMessages
    {
        Ball createBallFromROSMessage(const thunderbots_msgs::Ball& ball_msg)
        {
            Point ball_position  = Point(ball_msg.position.x, ball_msg.position.y);
            Vector ball_velocity = Vector(ball_msg.velocity.x, ball_msg.velocity.y);

            Ball ball = Ball(ball_position, ball_velocity);

            return ball;
        }

        Robot createRobotFromROSMessage(const thunderbots_msgs::Robot& robot_msg)
        {
            unsigned int robot_id   = robot_msg.id;
            Point robot_position    = Point(robot_msg.position.x, robot_msg.position.y);
            Vector robot_velocity   = Vector(robot_msg.velocity.x, robot_msg.velocity.y);
            Angle robot_orientation = Angle::ofRadians(robot_msg.orientation);
            AngularVelocity robot_angular_velocity =
                Angle::ofRadians(robot_msg.angular_velocity);
            auto epoch = std::chrono::steady_clock::time_point();
            auto since_epoch =
                std::chrono::nanoseconds(robot_msg.timestamp_nanoseconds_since_epoch);
            auto timestamp = epoch + since_epoch;

            Robot robot = Robot(robot_id, robot_position, robot_velocity,
                                robot_orientation, robot_angular_velocity, timestamp);

            return robot;
        }

        Field createFieldFromROSMessage(const thunderbots_msgs::Field& field_msg)
        {
            Field field = Field(field_msg.field_length, field_msg.field_width,
                                field_msg.defense_length, field_msg.defense_width,
                                field_msg.goal_width, field_msg.boundary_width,
                                field_msg.center_circle_radius);

            return field;
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
                std::chrono::milliseconds(team_msg.robot_expiry_buffer_milliseconds);

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

        RefboxGameState createGameStateFromROSMessage(
            const thunderbots_msgs::RefboxCommand& command)
        {
            // the values in RefboxGameState correspond to the constants in
            // RefboxCommand.msg
            return (RefboxGameState)command.command;
        }
    }  // namespace ROSMessages
}  // namespace Util
