#pragma once

#include "software/constants.h"
#include "software/networking/threaded_proto_multicast_listener.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/ssl_gc_referee_message.pb.h"

/**
 * This class encapsulates ProtoMulticastListener<SSLProto::SSL_WrapperPacket> and
 * ProtoMulticastListener<SSLProto::Referee> to abstract all ssl protobuf networking
 * operations behind a single interface.
 */
class SSLProtoClient
{
   public:
    /**
     * Creates a new SSLProtoClient
     *
     * @param received_vision_callback Callback for when a new SSLProto::SSL_WrapperPacket
     * is received
     * @param received_referee_callback Callback for when a new SSLProto::Referee is
     * received
     * @param ssl_communication_config The config defining the network parameters used
     * to communicate with the SSL applications
     */
    explicit SSLProtoClient(
        std::function<void(SSLProto::SSL_WrapperPacket)> received_vision_callback,
        std::function<void(SSLProto::Referee)> received_referee_callback,
        std::shared_ptr<const SslCommunicationConfig> ssl_communication_config);

   private:
    std::shared_ptr<const SslCommunicationConfig> ssl_communication_config;
    std::unique_ptr<ThreadedProtoMulticastListener<SSLProto::SSL_WrapperPacket>>
        ssl_vision_listener;
    std::unique_ptr<ThreadedProtoMulticastListener<SSLProto::Referee>>
        ssl_referee_listener;
};
