#include <ros/ros.h>
#include <iostream>
#include "backend_input/filter/ball_filter.h"
#include "backend_input/filter/robot_filter.h"
#include "backend_input/message_util.h"
#include "backend_input/vision_client/robocup_ssl_client.h"
#include "geom/point.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Team.h"

// TODO: Make a real constant
#define FRIENDLY_COLOR_YELLOW true

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "backend_input");
    ros::NodeHandle n;

    // Create publishers
    ros::Publisher ball_publisher =
        n.advertise<thunderbots_msgs::Ball>("backend/ball", 1);
    ros::Publisher field_publisher =
        n.advertise<thunderbots_msgs::Field>("backend/field", 1);
    ros::Publisher friendly_team_publisher =
        n.advertise<thunderbots_msgs::Team>("backend/friendly_team", 1);
    ros::Publisher enemy_team_publisher =
        n.advertise<thunderbots_msgs::Team>("backend/friendly_team", 1);

    // Set up the SSL Client to receive data over the network
    RoboCupSSLClient vision_client = RoboCupSSLClient(10020, "224.5.23.2");
    vision_client.open(true);
    SSL_WrapperPacket packet;

    // Set up any filters for the incoming data
    BallFilter ball_filter               = BallFilter();
    RobotTeamFilter friendly_team_filter = RobotTeamFilter();
    RobotTeamFilter enemy_team_filter    = RobotTeamFilter();

    // Main loop
    while (ros::ok())
    {
        if (vision_client.receive(packet))
        {
            if (packet.has_geometry())
            {
                const SSL_GeometryData &geom       = packet.geometry();
                const SSL_GeometryFieldSize &field = geom.field();
                thunderbots_msgs::Field field_msg  = VisionUtil::createFieldMsg(field);
                field_publisher.publish(field_msg);
            }

            if (packet.has_detection())
            {
                const SSL_DetectionFrame &detection = packet.detection();

                // Handle ball detection, filtering, and message updates
                std::vector<SSLBallData> ball_detections = std::vector<SSLBallData>();
                for (const SSL_DetectionBall &ball : detection.balls())
                {
                    SSLBallData ball_data;
                    ball_data.position   = Point(ball.x() / 1000.0, ball.y() / 1000.0);
                    ball_data.confidence = ball.confidence();
                    ball_detections.push_back(ball_data);
                }

                ball_filter.update(ball_detections);
                Point new_ball_position = ball_filter.getBallPosition();
                Point new_ball_velocity = ball_filter.getBallVelocity();
                thunderbots_msgs::Ball ball_msg =
                    VisionUtil::createBallMsg(new_ball_position, new_ball_velocity);
                ball_publisher.publish(ball_msg);


                // Handle Robot filtering, and updating the friendly and enemy teams
                std::vector<SSLRobotData> friendly_team_robot_data =
                    std::vector<SSLRobotData>();
                std::vector<SSLRobotData> enemy_team_robot_data =
                    std::vector<SSLRobotData>();

                for (const SSL_DetectionRobot &yellow_robot : detection.robots_yellow())
                {
                    SSLRobotData new_robot_data;
                    new_robot_data.id       = yellow_robot.robot_id();
                    new_robot_data.position = Point(yellow_robot.x(), yellow_robot.y());
                    new_robot_data.orientation =
                        Angle::ofRadians(yellow_robot.orientation());
                    new_robot_data.confidence = yellow_robot.confidence();

                    if (FRIENDLY_COLOR_YELLOW)
                    {
                        friendly_team_robot_data.emplace_back(new_robot_data);
                    }
                    else
                    {
                        enemy_team_robot_data.emplace_back(new_robot_data);
                    }
                }

                for (const SSL_DetectionRobot &blue_robot : detection.robots_blue())
                {
                    SSLRobotData new_robot_data;
                    new_robot_data.id       = blue_robot.robot_id();
                    new_robot_data.position = Point(blue_robot.x(), blue_robot.y());
                    new_robot_data.orientation =
                        Angle::ofRadians(blue_robot.orientation());
                    new_robot_data.confidence = blue_robot.confidence();

                    if (FRIENDLY_COLOR_YELLOW)
                    {
                        enemy_team_robot_data.emplace_back(new_robot_data);
                    }
                    else
                    {
                        friendly_team_robot_data.emplace_back(new_robot_data);
                    }
                }

                friendly_team_filter.update(friendly_team_robot_data);
                enemy_team_filter.update(enemy_team_robot_data);

                thunderbots_msgs::Team friendly_team_msg =
                    VisionUtil::createTeamMsg(friendly_team_filter.getFilteredTeamData());
                thunderbots_msgs::Team enemy_team_msg =
                    VisionUtil::createTeamMsg(enemy_team_filter.getFilteredTeamData());

                friendly_team_publisher.publish(friendly_team_msg);
                enemy_team_publisher.publish(enemy_team_msg);
            }
        }

        // We spin once here so any callbacks in this node can run (if we ever add them)
        ros::spinOnce();
    }

    return 0;
}
