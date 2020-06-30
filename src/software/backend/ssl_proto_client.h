#pragma once

#include "software/constants.h"
#include "software/networking/threaded_proto_multicast_listener.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/ssl_referee.pb.h"

/**
 * This class encapsulates ProtoMulticastListener<SSL_WrapperPacket> and
 * ProtoMulticastListener<SSL_Referee> to abstract all ssl protobuf networking operations
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
     * @param received_referee_callback Callback for when a new SSL_Referee is received
     * @param ssl_communication_config The config defining the network parameters used
     * to communicate with the SSL applications
     */
    explicit SSLProtoClient(
        std::function<void(SSL_WrapperPacket)> received_vision_callback,
        std::function<void(SSL_Referee)> received_referee_callback,
        std::shared_ptr<const SSLCommunicationConfig> ssl_communication_config);

   private:
    std::shared_ptr<const SSLCommunicationConfig> ssl_communication_config;
    std::unique_ptr<ThreadedProtoMulticastListener<SSL_WrapperPacket>>
        ssl_vision_listener;
    std::unique_ptr<ThreadedProtoMulticastListener<SSL_Referee>> ssl_referee_listener;
};
