#pragma once

#include <ros/ros.h>
#include <thunderbots_msgs/World.h>

#include <boost/asio.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <thread>

#include "network_input/backend.h"
#include "network_input/networking/ssl_gamecontroller_client.h"
#include "network_input/networking/ssl_vision_client.h"
#include "proto/messages_robocup_ssl_wrapper.pb.h"
#include "proto/ssl_referee.pb.h"

/**
 * This class encapsulates our SSLVisionClient and SSLGameController clients to abstract
 * all networking operations behind a single interface. This also allows us to keep the
 * "handle" functions we give to the clients as member functions rather than large lambda
 * functions. Overall, this helps keep our main.cpp file shorter and more readable.
 */
class NetworkClient
{
   public:
    /**
     * Creates a new NetworkClient for the given NodeHandle. This allows this class to
     * create and own its own publishers
     *
     * @param node_handle The NodeHandle this class should use to publish its messages
     */
    explicit NetworkClient(ros::NodeHandle& node_handle);

    /**
     * Safely destructs this NetworkClient object. Stops any running IO services and
     * gracefully joins the thread before exiting.
     */
    ~NetworkClient();

    // Delete the copy and assignment operators because this class really shouldn't need
    // them and we don't want to risk doing anything nasty with the internal threading
    // this class uses
    NetworkClient& operator=(const NetworkClient&) = delete;
    NetworkClient(const NetworkClient&)            = delete;

   private:
    // TODO: Remove this wrapper function once we move to a better simulator
    // https://github.com/UBC-Thunderbots/Software/issues/609
    /**
     * A wrapper function for the filterAndPublishVisionData function. This wrapper is
     * responsible for ignoring any bad packets we get from grSim, because grSim
     * sends garbage packets from very far in the future that causes issues if they
     * get through to our filters and logic.
     *
     * @param packet The vision packet
     */
    void filterAndPublishVisionDataWrapper(SSL_WrapperPacket packet);

    /**
     * Filters and publishes the new vision data
     *
     * This function contains all the work that is performed every time a new vision
     * packet is received from the network. We give this function to the SSLVisionClient
     * to call
     *
     * @param packet The newly received vision packet
     */
    void filterAndPublishVisionData(SSL_WrapperPacket packet);

    /**
     * Filters and publishes the new GameController data
     *
     * This function contains all the work that is performed every time a new
     * GameController packet is received from the network. We give this function to the
     * GameControllerClient to call
     *
     * @param packet The newly received GameController packet
     */
    void filterAndPublishGameControllerData(Referee packet);

    // The publishers used to send data after it has been received and processed
    ros::Publisher gamecontroller_publisher;
    ros::Publisher world_publisher;

    // The backend that handles data filtering and processing
    Backend backend;

    // The client that handles data reception, filtering, and publishing for vision data
    std::unique_ptr<SSLVisionClient> ssl_vision_client;
    // The client that handles data reception, filtering , and publishing for
    // gamecontroller data
    std::unique_ptr<SSLGameControllerClient> ssl_gamecontroller_client;

    // The most up-to-date state of the world
    thunderbots_msgs::World world_msg;

    // The io_service that will be used to serivce all network requests
    boost::asio::io_service io_service;

    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;

    // Both these values are used for the filterAndPublishVisionDataWrapper function
    // and should be removed when the function is removed
    // The t_capture of the latest SSL_WrapperPacket we received with a valid timestamp
    double last_valid_t_capture;
    // How many packets to analyze to find the true starting time of the vision system
    // before passing the packets on to the actual logic
    int initial_packet_count;
};
