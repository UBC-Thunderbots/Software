#include "software/backend/ssl_proto_client.h"

SSLProtoClient::SSLProtoClient(
    std::function<void(SSLProto::SSL_WrapperPacket)> received_vision_callback,
    std::function<void(SSLProto::Referee)> received_referee_callback,
    std::shared_ptr<const SslCommunicationConfig> ssl_communication_config)
    : ssl_communication_config(ssl_communication_config),
      ssl_vision_listener(
          std::make_unique<ThreadedProtoUdpListener<SSLProto::SSL_WrapperPacket>>(
              ssl_communication_config->getVisionIpv4Address()->value(),
              ssl_communication_config->getVisionPort()->value(),
              received_vision_callback, true)),
      ssl_referee_listener(std::make_unique<ThreadedProtoUdpListener<SSLProto::Referee>>(
          ssl_communication_config->getGameControllerIpv4Address()->value(),
          ssl_communication_config->getGameControllerPort()->value(),
          received_referee_callback, true))
{
}
