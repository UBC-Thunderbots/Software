#include <ros/ros.h>
#include <iostream>
#include "backend_input/filter/ball_filter.h"
#include "backend_input/vision_client/robocup_ssl_client.h"
#include "backend_input/message_util.h"
#include "geom/point.h"
#include "thunderbots_msgs/Ball.h"

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "backend_input");
    ros::NodeHandle n;

    // Create publishers
    ros::Publisher ball_publisher =
        n.advertise<thunderbots_msgs::Ball>("backend/ball", 1);

    // Set up the SSL Client to receive data over the network
    RoboCupSSLClient vision_client = RoboCupSSLClient(10020, "224.5.23.2");
    vision_client.open(true);
    SSL_WrapperPacket packet;

    // Set up any filters for the incoming data
    BallFilter ball_filter = BallFilter();

    // Main loop
    while (ros::ok())
    {
        if (vision_client.receive(packet))
        {
            if (packet.has_geometry())
            {
                const SSL_GeometryData &geom       = packet.geometry();
                const SSL_GeometryFieldSize &field = geom.field();
                std::cout << "Field boundary width is: " << field.boundary_width()
                          << std::endl;
            }

            if (packet.has_detection())
            {
                const SSL_DetectionFrame &detection = packet.detection();

                // Handle ball detection, filtering, and message updates
                std::vector<SSLBallData> ball_detections = std::vector<SSLBallData>();
                for (const SSL_DetectionBall &ball : detection.balls())
                {
                    SSLBallData ball_data;
                    ball_data.position   = Point(ball.x(), ball.y());
                    ball_data.confidence = ball.confidence();
                    ball_detections.push_back(ball_data);
                }

                ball_filter.update(ball_detections);
                Point new_ball_position = ball_filter.getBallPosition();
                Point new_ball_velocity = ball_filter.getBallVelocity();
                thunderbots_msgs::Ball ball_msg =
                    VisionUtil::createBallMsg(new_ball_position, new_ball_velocity);
                ball_publisher.publish(ball_msg);
            }
        }

        // We spin once here so any callbacks in this node can run (if we ever add them)
        ros::spinOnce();
    }

    return 0;
}
