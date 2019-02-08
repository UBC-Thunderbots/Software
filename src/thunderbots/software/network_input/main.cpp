#include <ros/ros.h>

#include <boost/exception/diagnostic_information.hpp>

#include "geom/point.h"
#include "network_input/backend.h"
#include "network_input/networking/ssl_gamecontroller_client.h"
#include "network_input/networking/ssl_vision_client.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Team.h"
#include "thunderbots_msgs/World.h"
#include "util/constants.h"
#include "util/logger/init.h"
#include "util/ros_messages.h"
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
    ros::Publisher world_publisher = node_handle.advertise<thunderbots_msgs::World>(
        Util::Constants::NETWORK_INPUT_WORLD_TOPIC, 1);

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

    // Store the state of the most up to date World message
    thunderbots_msgs::World world_msg;

    // Main loop
    while (ros::ok())
    {
        auto ssl_vision_packet_queue = ssl_vision_client->getVisionPacketQueue();
        auto gamecontroller_packet_ptr =
            ssl_gamecontroller_client->getGameControllerPacket();

        if (!ssl_vision_packet_queue.empty() || gamecontroller_packet_ptr)
        {
            while (!ssl_vision_packet_queue.empty())
            {
                auto ssl_vision_packet = ssl_vision_packet_queue.front();
                ssl_vision_packet_queue.pop();

                Field field     = backend.getFieldData(ssl_vision_packet);
                world_msg.field = Util::ROSMessages::convertFieldToROSMessage(field);

                Ball ball      = backend.getFilteredBallData(ssl_vision_packet);
                world_msg.ball = Util::ROSMessages::convertBallToROSMessage(ball);

                Team friendly_team =
                    backend.getFilteredFriendlyTeamData(ssl_vision_packet);
                world_msg.friendly_team =
                    Util::ROSMessages::convertTeamToROSMessage(friendly_team);

                Team enemy_team = backend.getFilteredEnemyTeamData(ssl_vision_packet);
                world_msg.enemy_team =
                    Util::ROSMessages::convertTeamToROSMessage(enemy_team);
            }

            if (gamecontroller_packet_ptr)
            {
                auto gamecontroller_data_msg =
                    backend.getRefboxDataMsg(*gamecontroller_packet_ptr);
                if (gamecontroller_data_msg)
                {
                    world_msg.refbox_data = *gamecontroller_data_msg;
                }
            }

            ball_publisher.publish(world_msg.ball);
            field_publisher.publish(world_msg.field);
            friendly_team_publisher.publish(world_msg.friendly_team);
            enemy_team_publisher.publish(world_msg.enemy_team);
            gamecontroller_publisher.publish(world_msg.refbox_data);
            world_publisher.publish(world_msg);
        }

        // We spin once here so any callbacks in this node can run (if we ever add them)
        ros::spinOnce();
    }

    return 0;
}
