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
    ros::Publisher gamecontroller_publisher =
        node_handle.advertise<thunderbots_msgs::RefboxData>(
            Util::Constants::NETWORK_INPUT_GAMECONTROLLER_TOPIC, 1);
    ros::Publisher world_publisher = node_handle.advertise<thunderbots_msgs::World>(
        Util::Constants::NETWORK_INPUT_WORLD_TOPIC, 1);

    // Initialize the logger
    Util::Logger::LoggerSingleton::initializeLogger(node_handle);

    // Init our backend class
    Backend backend = Backend();

    // Store the state of the most up to date World message
    thunderbots_msgs::World world_msg;

    // Create the io_service that will be used to service all network requests
    boost::asio::io_service io_service;

    // The function that will be run on every received SSL_WrapperPacket
    auto filter_and_publish_vision_data = [&backend, world_publisher,
                                           &world_msg](SSL_WrapperPacket packet) {
        // A map used to store the latest detection data for each camera
        static std::map<unsigned int, SSL_DetectionFrame> latest_detection_data;

        if (packet.has_geometry())
        {
            const auto& latest_geometry_data = packet.geometry();
            Field field                      = backend.getFieldData(latest_geometry_data);
            thunderbots_msgs::Field field_msg =
                Util::ROSMessages::convertFieldToROSMessage(field);
            world_msg.field = field_msg;
        }

        if (packet.has_detection())
        {
            auto detection = packet.detection();
            // add the detection to the map, replacing any existing values
            auto ret = latest_detection_data.insert(
                std::make_pair(detection.camera_id(), detection));
            // check if we inserted successfully. if not, modify the existing
            // entry to be our new value
            if (!ret.second)
            {
                ret.first->second = detection;
            }

            // Create a vector of the latest detection data for each camera
            std::vector<SSL_DetectionFrame> latest_detections;
            for (auto it = latest_detection_data.begin();
                 it != latest_detection_data.end(); it++)
            {
                latest_detections.push_back(it->second);
            }

            Ball ball = backend.getFilteredBallData(latest_detections);
            thunderbots_msgs::Ball ball_msg =
                Util::ROSMessages::convertBallToROSMessage(ball);
            world_msg.ball = ball_msg;

            Team friendly_team = backend.getFilteredFriendlyTeamData(latest_detections);
            thunderbots_msgs::Team friendly_team_msg =
                Util::ROSMessages::convertTeamToROSMessage(friendly_team);
            world_msg.friendly_team = friendly_team_msg;

            Team enemy_team = backend.getFilteredEnemyTeamData(latest_detections);
            thunderbots_msgs::Team enemy_team_msg =
                Util::ROSMessages::convertTeamToROSMessage(enemy_team);
            world_msg.enemy_team = enemy_team_msg;
        }

        world_publisher.publish(world_msg);
    };

    // Set up our connection over udp to receive camera packets
    // NOTE: We do this before initializing the ROS node so that if it
    // fails because there is another instance of this node running
    // and connected to the port we want, we don't kill that other node.
    std::unique_ptr<SSLVisionClient> ssl_vision_client;
    try
    {
        ssl_vision_client = std::make_unique<SSLVisionClient>(
            io_service, Util::Constants::SSL_VISION_MULTICAST_ADDRESS,
            Util::Constants::SSL_VISION_MULTICAST_PORT, filter_and_publish_vision_data);
    }
    catch (const boost::exception& ex)
    {
        // LOG(FATAL) will terminate the network_input process
        LOG(FATAL) << "An error occured while setting up the SSL Vision Client:"
                   << std::endl
                   << boost::diagnostic_information(ex) << std::endl;
    }

    // The function that will be run on every received Referee packet
    auto filter_and_publish_gamecontroller_data = [&backend, gamecontroller_publisher,
                                                   &world_msg](Referee packet) {
        auto gamecontroller_data_msg = backend.getRefboxDataMsg(packet);
        gamecontroller_publisher.publish(gamecontroller_data_msg);
        world_msg.refbox_data = gamecontroller_data_msg;
    };

    // Set up our connection over udp to receive gamecontroller packets
    // NOTE: We do this before initializing the ROS node so that if it
    // fails because there is another instance of this node running
    // and connected to the port we want, we don't kill that other node.
    std::unique_ptr<SSLGameControllerClient> ssl_gamecontroller_client;
    try
    {
        ssl_gamecontroller_client = std::make_unique<SSLGameControllerClient>(
            io_service, Util::Constants::SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
            Util::Constants::SSL_GAMECONTROLLER_MULTICAST_PORT,
            filter_and_publish_gamecontroller_data);
    }
    catch (const boost::exception& ex)
    {
        // LOG(FATAL) will terminate the network_input process
        LOG(FATAL) << "An error occured while setting up the SSL GameController Client:"
                   << std::endl
                   << boost::diagnostic_information(ex) << std::endl;
    }

    // Run the io_service in a separate thread so it doesn't block the ros::spin()
    // call. This runs the IO services for BOTH the SSLVisionClient and
    // SSLGameControllerClient
    std::thread network_thread([&io_service]() { io_service.run(); });

    // Services any ROS calls in a separate thread "behind the scenes". Does not return
    // until the node is shutdown
    // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::spin();

    return 0;
}
