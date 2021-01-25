#include "software/backend/ssl_proto_client.h"

SSLProtoClient::SSLProtoClient(
    std::function<void(SSLProto::SSL_WrapperPacket)> received_vision_callback,
    std::function<void(SSLProto::Referee)> received_referee_callback,
    std::shared_ptr<const SSLCommunicationConfig> ssl_communication_config)
    : ssl_communication_config(ssl_communication_config),
      ssl_vision_listener(
          std::make_unique<ThreadedProtoMulticastListener<SSLProto::SSL_WrapperPacket>>(
              ssl_communication_config->VisionIpv4Address()->value(),
              ssl_communication_config->VisionPort()->value(), received_vision_callback)),
      ssl_referee_listener(
          std::make_unique<ThreadedProtoMulticastListener<SSLProto::Referee>>(
              ssl_communication_config->GameControllerIpv4Address()->value(),
              ssl_communication_config->GameControllerPort()->value(),
              received_referee_callback))
{
}
