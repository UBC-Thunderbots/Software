#include "software/backend/ssl_proto_client.h"

SSLProtoClient::SSLProtoClient(
    std::function<void(SSL_WrapperPacket)> received_vision_callback,
    std::function<void(SSL_Referee)> received_referee_callback,
    std::shared_ptr<const SSLCommunicationConfig> ssl_communication_config)
    : ssl_communication_config(ssl_communication_config),
      ssl_vision_listener(
          std::make_unique<ThreadedProtoMulticastListener<SSL_WrapperPacket>>(
              ssl_communication_config->VisionIPv4Address()->value(),
              ssl_communication_config->VisionPort()->value(), received_vision_callback)),
      ssl_referee_listener(std::make_unique<ThreadedProtoMulticastListener<SSL_Referee>>(
          ssl_communication_config->GamecontrollerIPv4Address()->value(),
          ssl_communication_config->GamecontrollerPort()->value(),
          received_referee_callback))
{
}
