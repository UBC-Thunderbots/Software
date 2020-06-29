#pragma once

#include "software/constants.h"
#include "software/networking/threaded_proto_multicast_listener.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/ssl_referee.pb.h"

/**
 * This class encapsulates ProtoMulticastListener<SSL_WrapperPacket> and
 * ProtoMulticastListener<Referee> to abstract all ssl protobuf networking operations
 * behind a single interface.
 */
class SSLProtoClient
{
   public:
    /**
     * Creates a new SSLProtoClient
     *
     * @param received_vision_callback Callback for when a new SSL_WrapperPacket is
     * received
     * @param received_referee_callback Callback for when a new Referee is received
     * @param vision_multicast_address IP address the vision system is running on
     * @param vision_multicast_port The port the vision system is running on
     * @param gamecontroller_multicast_address IP address the gamecontroller system is
     * running on
     * @param gamecontroller_multicast_port The port the gamecontroller is running on
     */
    explicit SSLProtoClient(
        std::function<void(SSL_WrapperPacket)> received_vision_callback,
        std::function<void(Referee)> received_referee_callback,
        std::string vision_multicast_address = SSL_VISION_DEFAULT_MULTICAST_ADDRESS,
        int vision_multicast_port            = SSL_VISION_MULTICAST_PORT,
        std::string gamecontroller_multicast_address =
            SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
        int gamecontroller_multicast_port = SSL_GAMECONTROLLER_MULTICAST_PORT);

   private:
    // The client that handles data reception, filtering, and publishing for vision data
    std::unique_ptr<ThreadedProtoMulticastListener<SSL_WrapperPacket>>
        ssl_vision_listener;
    // The client that handles data reception, filtering , and publishing for referee data
    std::unique_ptr<ThreadedProtoMulticastListener<Referee>> ssl_referee_listener;
};
