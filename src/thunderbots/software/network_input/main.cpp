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
    // A map used to store the latest detection data for each camera
    std::map<unsigned int, SSL_DetectionFrame> latest_detection_data;
    // Stores the most recent geometry data received
    SSL_GeometryData latest_geometry_data;

    // Main loop
    while (ros::ok())
    {
        auto ssl_vision_packets = ssl_vision_client->getVisionPacketVector();
        for (const auto& packet : ssl_vision_packets)
        {
            if (packet.has_geometry())
            {
                latest_geometry_data = packet.geometry();
            }

            if (packet.has_detection())
            {
                auto detection = packet.detection();
                // Add the detection to the map, replacing any existing values
                auto ret = latest_detection_data.insert(
                    std::make_pair(detection.camera_id(), detection));
                // Check if we inserted successfully. If not, modify the existing
                // entry to be our new value
                if (!ret.second)
                {
                    ret.first->second = detection;
                }
            }
        }

        // Create a vector of the latest detection data for each camera
        std::vector<SSL_DetectionFrame> latest_detections;
        for (auto it = latest_detection_data.begin(); it != latest_detection_data.end();
             it++)
        {
            latest_detections.push_back(it->second);
        }

        auto gamecontroller_packet_ptr =
            ssl_gamecontroller_client->getGameControllerPacket();

        // Update data and publish the World message if any data changed
        if (!ssl_vision_packets.empty() || gamecontroller_packet_ptr)
        {
            if (!ssl_vision_packets.empty())
            {
                Field field = backend.getFieldData(latest_geometry_data);
                thunderbots_msgs::Field field_msg =
                    Util::ROSMessages::convertFieldToROSMessage(field);
                field_publisher.publish(field_msg);
                world_msg.field = field_msg;

                Ball ball = backend.getFilteredBallData(latest_detections);
                thunderbots_msgs::Ball ball_msg =
                    Util::ROSMessages::convertBallToROSMessage(ball);
                ball_publisher.publish(ball_msg);
                world_msg.ball = ball_msg;

                Team friendly_team =
                    backend.getFilteredFriendlyTeamData(latest_detections);
                thunderbots_msgs::Team friendly_team_msg =
                    Util::ROSMessages::convertTeamToROSMessage(friendly_team);
                friendly_team_publisher.publish(friendly_team_msg);
                world_msg.friendly_team = friendly_team_msg;

                Team enemy_team = backend.getFilteredEnemyTeamData(latest_detections);
                thunderbots_msgs::Team enemy_team_msg =
                    Util::ROSMessages::convertTeamToROSMessage(enemy_team);
                enemy_team_publisher.publish(enemy_team_msg);
                world_msg.enemy_team = enemy_team_msg;
            }

            if (gamecontroller_packet_ptr)
            {
                auto gamecontroller_data_msg =
                    backend.getRefboxDataMsg(*gamecontroller_packet_ptr);
                if (gamecontroller_data_msg)
                {
                    gamecontroller_publisher.publish(*gamecontroller_data_msg);
                    world_msg.refbox_data = *gamecontroller_data_msg;
                }
            }

            world_publisher.publish(world_msg);
        }

        // We spin once here so any callbacks in this node can run (if we ever add them)
        ros::spinOnce();
    }

    return 0;
}
