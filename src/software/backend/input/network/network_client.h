#pragma once

#include <boost/asio.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <thread>

#include "software/backend/input/network/ssl_gamecontroller_client.h"
#include "software/backend/input/network/ssl_protobuf_reader.h"
#include "software/backend/input/network/ssl_vision_client.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/ssl_referee.pb.h"
#include "software/sensor_fusion/vision_detection.h"
#include "software/world/world.h"

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
     * Creates a new NetworkClient. This class creates and owns its own publishers
     *
     * @param vision_multicast_address A string representation of the ip address the
     *                                 vision system is running on
     * @param vision_multicast_port The port the vision system is running on
     * @param gamecontroller_multicast_address A string representation of the ip address
     *                                         the gamecontroller is running on
     * @param gamecontroller_multicast_port The port the gamecontroller is running on
     * @param received_vision_detection_callback This function will be called with a new
     * VisionDetection every time one is received
     * @param received_refbox_data_callback This function will be called with a new
     * RefboxData every time one is received
     */
    explicit NetworkClient(
        std::string vision_multicast_address, int vision_multicast_port,
        std::string gamecontroller_multicast_address, int gamecontroller_multicast_port,
        std::function<void(VisionDetection)> received_vision_detection_callback,
        std::function<void(RefboxData)> received_refbox_data_callback);

    /**
     * Safely destructs this NetworkClient object. Stops any running IO services and
     * gracefully joins the thread before exiting.
     */
    ~NetworkClient();

    // Delete the copy and assignment operators because this class really shouldn't need
    // them and we don't want to risk doing anything nasty with the internal
    // threading this class uses
    NetworkClient& operator=(const NetworkClient&) = delete;
    NetworkClient(const NetworkClient&)            = delete;

   private:
    /**
     * Sets up the vision client to the point where it will receive packets on the
     * given address/port
     *
     * @param vision_address String representation of the vision IP address
     * @param vision_port The port vision is running on
     */
    void setupVisionClient(std::string vision_address, int vision_port);

    /**
     * Sets up the gamecontroller client to the point where it will receive packets on the
     * given address/port
     *
     * @param gamecontroller_address String representation of the gamecontroller IP
     * address
     * @param gamecontroller_port The port gamecontroller is running on
     */
    void setupGameControllerClient(std::string gamecontroller_address,
                                   int gamecontroller_port);

    /**
     * Starts up the IO service thread to run and service network requests in the
     * background
     */
    void startIoServiceThreadInBackground();

    /**
     * Given a detection, figures out if the camera is enabled
     *
     * @param detection SSL_DetectionFrame to consider
     *
     * @return whether the camera is enabled
     */
    bool isCameraEnabled(const SSL_DetectionFrame& detection);

    // TODO: Remove this wrapper function once we move to a better simulator
    // https://github.com/UBC-Thunderbots/Software/issues/609
    /**
     * A wrapper function for the publishVisionData function. This wrapper is
     * responsible for ignoring any bad packets we get from grSim, because grSim
     * sends garbage packets from very far in the future that causes issues if they
     * get through to our filters and logic.
     *
     * @param packet The vision packet
     */
    void publishVisionDataWrapper(SSL_WrapperPacket packet);

    /**
     * Publishes the new vision data
     *
     * This function contains all the work that is performed every time a new vision
     * packet is received from the network. We give this function to the SSLVisionClient
     * to call
     *
     * @param packet The newly received vision packet
     */
    void publishVisionData(SSL_WrapperPacket packet);

    /**
     * Publishes the new GameController data
     *
     * This function contains all the work that is performed every time a new
     * GameController packet is received from the network. We give this function to the
     * GameControllerClient to call
     *
     * @param packet The newly received GameController packet
     */
    void publishGameControllerData(Referee packet);

    /**
     * Inverts all positions and orientations across the x and y axis of the field
     *
     * @param frame The frame to invert. It will be mutated in-place
     */
    static void invertFieldSide(SSL_DetectionFrame& frame);

    // The backend that handles reading and processing protobuf messages
    SSLProtobufReader ssl_protobuf_reader;

    // The client that handles data reception and publishing for vision data
    std::unique_ptr<SSLVisionClient> ssl_vision_client;
    // The client that handles data reception and publishing for
    // gamecontroller data
    std::unique_ptr<SSLGameControllerClient> ssl_gamecontroller_client;

    // The most up-to-date state of the world
    World world;

    // The io_service that will be used to serivce all network requests
    boost::asio::io_service io_service;

    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;

    // Both these values are used for the publishVisionDataWrapper function
    // and should be removed when the function is removed
    // The t_capture of the latest SSL_WrapperPacket we received with a valid timestamp
    double last_valid_t_capture;
    // How many packets to analyze to find the true starting time of the vision system
    // before passing the packets on to the actual logic
    int initial_packet_count;

    // The callback function that we pass newly received VisionDetection to
    std::function<void(VisionDetection)> received_vision_detection_callback;

    // The callback function that we pass newly received RefboxData to
    std::function<void(RefboxData)> received_refbox_data_callback;
};
