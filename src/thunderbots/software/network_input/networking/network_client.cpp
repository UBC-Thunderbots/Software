#include "network_input/networking/network_client.h"

#include <boost/bind.hpp>

#include "util/constants.h"
#include "util/logger/init.h"
#include "util/ros_messages.h"

NetworkClient::NetworkClient(ros::NodeHandle& node_handle) : backend(), io_service()
{
    // Set up publishers
    world_publisher = node_handle.advertise<thunderbots_msgs::World>(
        Util::Constants::NETWORK_INPUT_WORLD_TOPIC, 1);
    gamecontroller_publisher = node_handle.advertise<thunderbots_msgs::RefboxData>(
        Util::Constants::NETWORK_INPUT_GAMECONTROLLER_TOPIC, 1);

    // Set up our connection over udp to receive vision packets
    try
    {
        ssl_vision_client = std::make_unique<SSLVisionClient>(
            io_service, Util::Constants::SSL_VISION_MULTICAST_ADDRESS,
            Util::Constants::SSL_VISION_MULTICAST_PORT,
            boost::bind(&NetworkClient::filterAndPublishVisionData, this, _1));
    }
    catch (const boost::exception& ex)
    {
        // LOG(FATAL) will terminate the network_input process
        LOG(FATAL) << "An error occured while setting up the SSL Vision Client:"
                   << std::endl
                   << boost::diagnostic_information(ex) << std::endl;
    }


    // Set up our connection over udp to receive gamecontroller packets
    try
    {
        ssl_gamecontroller_client = std::make_unique<SSLGameControllerClient>(
            io_service, Util::Constants::SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
            Util::Constants::SSL_GAMECONTROLLER_MULTICAST_PORT,
            boost::bind(&NetworkClient::filterAndPublishGameControllerData, this, _1));
    }
    catch (const boost::exception& ex)
    {
        // LOG(FATAL) will terminate the network_input process
        LOG(FATAL) << "An error occured while setting up the SSL GameController Client:"
                   << std::endl
                   << boost::diagnostic_information(ex) << std::endl;
    }

    // Start the thread to run the io_service in the background
    io_service_thread = std::thread([this]() { io_service.run(); });
}

NetworkClient::~NetworkClient()
{
    // Stop the io_service. This is safe to call from another thread.
    // https://stackoverflow.com/questions/4808848/boost-asio-stopping-io-service
    // This MUST be done before attempting to join the thread because otherwise the
    // io_service will not stop and the thread will not join
    io_service.stop();

    // Join the io_service_thread so that we wait for it to exit before destructing the
    // thread object. If we do not wait for the thread to finish executing, it will call
    // `std::terminate` when we deallocate the thread object and kill our whole program
    io_service_thread.join();
}


void NetworkClient::filterAndPublishVisionData(SSL_WrapperPacket packet)
{
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
        for (auto it = latest_detection_data.begin(); it != latest_detection_data.end();
             it++)
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
}

void NetworkClient::filterAndPublishGameControllerData(Referee packet)
{
    auto gamecontroller_data_msg = backend.getRefboxDataMsg(packet);
    world_msg.refbox_data        = gamecontroller_data_msg;
    world_publisher.publish(world_msg);
}
