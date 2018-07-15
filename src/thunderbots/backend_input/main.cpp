#include <ros/ros.h>
#include <iostream>
#include "backend_input/vision_client/robocup_ssl_client.h"

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "backend_input");
    ros::NodeHandle n;

    // Set up the SSL Client to receive data over the network
    RoboCupSSLClient vision_client = RoboCupSSLClient(10020, "224.5.23.2");
    vision_client.open(true);
    SSL_WrapperPacket packet;

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
                std::cout << "Number of balls detected: " << detection.balls_size()
                          << std::endl;
            }
        }

        // We spin once here so any callbacks in this node can run (if we ever add them)
        ros::spinOnce();
    }

    return 0;
}
