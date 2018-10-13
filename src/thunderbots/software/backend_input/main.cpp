#include <ros/ros.h>

#include "backend_input/backend.h"
#include "backend_input/networking/ssl_vision_client.h"
#include "geom/point.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Team.h"
#include "util/constants.h"
#include "util/timestamp.h"


int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "backend_input");
    ros::NodeHandle node_handle;

    // Create publishers
    ros::Publisher ball_publisher = node_handle.advertise<thunderbots_msgs::Ball>(
        UTIL::CONSTANTS::BACKEND_INPUT_BALL_TOPIC, 1);
    ros::Publisher field_publisher = node_handle.advertise<thunderbots_msgs::Field>(
        UTIL::CONSTANTS::BACKEND_INPUT_FIELD_TOPIC, 1);
    ros::Publisher friendly_team_publisher =
        node_handle.advertise<thunderbots_msgs::Team>(
            UTIL::CONSTANTS::BACKEND_INPUT_FRIENDLY_TEAM_TOPIC, 1);
    ros::Publisher enemy_team_publisher = node_handle.advertise<thunderbots_msgs::Team>(
        UTIL::CONSTANTS::BACKEND_INPUT_ENEMY_TEAM_TOPIC, 1);

    // Set up our backend
    Backend backend = Backend();
    SSLVisionClient ssl_vision_client =
        SSLVisionClient(UTIL::CONSTANTS::SSL_VISION_MULTICAST_ADDRESS,
                        UTIL::CONSTANTS::SSL_VISION_MULTICAST_PORT);

    // Main loop
    while (ros::ok())
    {
        auto ssl_vision_packet_ptr = ssl_vision_client.getVisionPacket();
        if (ssl_vision_packet_ptr)
        {
            auto ssl_vision_packet = *ssl_vision_packet_ptr;

            AITimestamp timestamp = Timestamp::getTimestampNow();

            std::optional<thunderbots_msgs::Field> field_msg =
                backend.getFieldMsg(ssl_vision_packet);
            if (field_msg)
            {
                field_publisher.publish(*field_msg);
            }

            std::optional<thunderbots_msgs::Ball> ball_msg =
                backend.getFilteredBallMsg(ssl_vision_packet, timestamp);
            if (ball_msg)
            {
                ball_publisher.publish(*ball_msg);
            }

            std::optional<thunderbots_msgs::Team> friendly_team_msg =
                backend.getFilteredFriendlyTeamMsg(ssl_vision_packet, timestamp);
            if (friendly_team_msg)
            {
                friendly_team_publisher.publish(*friendly_team_msg);
            }

            std::optional<thunderbots_msgs::Team> enemy_team_msg =
                backend.getFilteredEnemyTeamMsg(ssl_vision_packet, timestamp);
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
