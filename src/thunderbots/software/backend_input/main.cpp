#include <ros/ros.h>
#include <iostream>
#include "../shared_util/constants.h"
#include "backend_input/backend.h"
#include "backend_input/vision_client/robocup_ssl_client.h"
#include "geom/point.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Team.h"
#include "util/timestamp.h"

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "backend_input");
    ros::NodeHandle node_handle;

    // Create publishers
    ros::Publisher ball_publisher =
        node_handle.advertise<thunderbots_msgs::Ball>(BACKEND_INPUT_BALL_TOPIC, 1);
    ros::Publisher field_publisher =
        node_handle.advertise<thunderbots_msgs::Field>(BACKEND_INPUT_FIELD_TOPIC, 1);
    ros::Publisher friendly_team_publisher =
        node_handle.advertise<thunderbots_msgs::Team>(
            BACKEND_INPUT_FRIENDLY_TEAM_TOPIC, 1);
    ros::Publisher enemy_team_publisher =
        node_handle.advertise<thunderbots_msgs::Team>(BACKEND_INPUT_ENEMY_TEAM_TOPIC, 1);

    // Set up our backend
    Backend backend = Backend();

    // Set up the SSL Client to receive data over the network
    RoboCupSSLClient vision_client = RoboCupSSLClient(10020, "224.5.23.2");
    vision_client.open(true);
    SSL_WrapperPacket packet;

    // Main loop
    while (ros::ok())
    {
        if (vision_client.receive(packet))
        {
            AITimestamp timestamp = Timestamp::getTimestampNow();

            std::optional<thunderbots_msgs::Field> field_msg =
                backend.getFieldMsg(packet);
            if (field_msg)
            {
                field_publisher.publish(*field_msg);
            }

            std::optional<thunderbots_msgs::Ball> ball_msg =
                backend.getFilteredBallMsg(packet, timestamp);
            if (ball_msg)
            {
                field_publisher.publish(*ball_msg);
            }

            std::optional<thunderbots_msgs::Team> friendly_team_msg =
                backend.getFilteredFriendlyTeamMsg(packet, timestamp);
            if (friendly_team_msg)
            {
                friendly_team_publisher.publish(*friendly_team_msg);
            }

            std::optional<thunderbots_msgs::Team> enemy_team_msg =
                backend.getFilteredEnemyTeamMsg(packet, timestamp);
            if (enemy_team_msg)
            {
                enemy_team_publisher.publish(*enemy_team_msg);
            }
        }

        // We spin once here so any callbacks in this node can run (if we ever add them)
        ros::spinOnce();
    }

    return 0;
}
