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

            Robot robot =
                Robot(robot_id, robot_position, robot_velocity, robot_orientation,
                      robot_angular_velocity, std::chrono::steady_clock::now());

            return robot;
        }

        Field createFieldFromROSMessage(const thunderbots_msgs::Field& field_msg)
        {
            Field field = Field();

            field.updateDimensions(field_msg.field_length, field_msg.field_width,
                                   field_msg.defense_length, field_msg.defense_width,
                                   field_msg.goal_width, field_msg.boundary_width,
                                   field_msg.center_circle_radius);

            return field;
        }
    }
}