#include <ros/ros.h>

#include <boost/exception/diagnostic_information.hpp>

#include "geom/point.h"
#include "network_input/backend.h"
#include "network_input/networking/ssl_gamecontroller_client.h"
#include "network_input/networking/ssl_vision_client.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Team.h"
#include "util/constants.h"
#include "util/logger/init.h"
#include "util/timestamp.h"

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "network_input");
    ros::NodeHandle node_handle;

    // Create publishers
    // We give the publishers queue sizes equal to the number of cameras being used with
    // the SSL Vision system. This is to be able to buffer data from each camera if we are
    // receiving data faster than we can publish. This way the buffered data will contain
    // data for the entire field (something from each camera) and we don't lose any
    // information
    ros::Publisher ball_publisher = node_handle.advertise<thunderbots_msgs::Ball>(
        Util::Constants::NETWORK_INPUT_BALL_TOPIC,
        Util::Constants::NUMBER_OF_SSL_VISION_CAMERAS);
    ros::Publisher field_publisher = node_handle.advertise<thunderbots_msgs::Field>(
        Util::Constants::NETWORK_INPUT_FIELD_TOPIC,
        Util::Constants::NUMBER_OF_SSL_VISION_CAMERAS);
    ros::Publisher friendly_team_publisher =
        node_handle.advertise<thunderbots_msgs::Team>(
            Util::Constants::NETWORK_INPUT_FRIENDLY_TEAM_TOPIC,
            Util::Constants::NUMBER_OF_SSL_VISION_CAMERAS);
    ros::Publisher enemy_team_publisher = node_handle.advertise<thunderbots_msgs::Team>(
        Util::Constants::NETWORK_INPUT_ENEMY_TEAM_TOPIC,
        Util::Constants::NUMBER_OF_SSL_VISION_CAMERAS);
    ros::Publisher gamecontroller_publisher =
        node_handle.advertise<thunderbots_msgs::RefboxData>(
            Util::Constants::NETWORK_INPUT_GAMECONTROLLER_TOPIC, 1);

    // Initialize the logger
    Util::Logger::LoggerSingleton::initializeLogger(node_handle);

    // Set up our connection over udp to receive camera packets
    // NOTE: We do this before initializing the ROS node so that if it
    // fails because there is another instance of this node running
    // and connected to the port we want, we don't kill that other node.

    std::unique_ptr<SSLVisionClient> ssl_vision_client;
    try
    {
        ssl_vision_client = std::make_unique<SSLVisionClient>(
            Util::Constants::SSL_VISION_MULTICAST_ADDRESS,
            Util::Constants::SSL_VISION_MULTICAST_PORT);
    }
    catch (const boost::exception& ex)
    {
        // LOG(FATAL) will terminate the network_input process
        LOG(FATAL) << "An error occured while setting up the SSL Vision Client:"
                   << std::endl
                   << boost::diagnostic_information(ex) << std::endl;
    }

    std::unique_ptr<SSLGameControllerClient> ssl_gamecontroller_client;
    try
    {
        ssl_gamecontroller_client = std::make_unique<SSLGameControllerClient>(
            Util::Constants::SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
            Util::Constants::SSL_GAMECONTROLLER_MULTICAST_PORT);
    }
    catch (const boost::exception& ex)
    {
        // LOG(FATAL) will terminate the network_input process
        LOG(FATAL) << "An error occured while setting up the SSL Game Controller Client:"
                   << std::endl
                   << boost::diagnostic_information(ex) << std::endl;
    }

    // Init our backend class
    Backend backend = Backend();

    // Main loop
    while (ros::ok())
    {
        auto ssl_vision_packet_queue = ssl_vision_client->getVisionPacketQueue();
        while (!ssl_vision_packet_queue.empty())
        {
            auto ssl_vision_packet = ssl_vision_packet_queue.front();
            ssl_vision_packet_queue.pop();

            std::optional<thunderbots_msgs::Field> field_msg =
                backend.getFieldMsg(ssl_vision_packet);
            if (field_msg)
            {
                field_publisher.publish(*field_msg);
            }

            std::optional<thunderbots_msgs::Ball> ball_msg =
                backend.getFilteredBallMsg(ssl_vision_packet);
            if (ball_msg)
            {
                ball_publisher.publish(*ball_msg);
            }

            std::optional<thunderbots_msgs::Team> friendly_team_msg =
                backend.getFilteredFriendlyTeamMsg(ssl_vision_packet);
            if (friendly_team_msg)
            {
                friendly_team_publisher.publish(*friendly_team_msg);
            }

            std::optional<thunderbots_msgs::Team> enemy_team_msg =
                backend.getFilteredEnemyTeamMsg(ssl_vision_packet);
            if (enemy_team_msg)
            {
                enemy_team_publisher.publish(*enemy_team_msg);
            }
        }

        auto gamecontroller_packet_ptr =
            ssl_gamecontroller_client->getGameControllerPacket();

        if (gamecontroller_packet_ptr)
        {
            auto gamecontroller_data_msg =
                backend.getRefboxDataMsg(*gamecontroller_packet_ptr);
            if (gamecontroller_data_msg)
            {
                gamecontroller_publisher.publish(*gamecontroller_data_msg);
            }
        }

        // We spin once here so any callbacks in this node can run (if we ever add them)
        ros::spinOnce();
    }

    return 0;
}
