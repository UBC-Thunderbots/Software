#include "software/backend/ssl_proto_client.h"

SSLProtoClient::SSLProtoClient(
    std::function<void(SSL_WrapperPacket)> received_vision_callback,
    std::function<void(SSL_Referee)> received_referee_callback,
    std::string vision_multicast_address, int vision_multicast_port,
    std::string gamecontroller_multicast_address, int gamecontroller_multicast_port)
    : ssl_vision_listener(
          std::make_unique<ThreadedProtoMulticastListener<SSL_WrapperPacket>>(
              vision_multicast_address, vision_multicast_port, received_vision_callback)),
      ssl_referee_listener(std::make_unique<ThreadedProtoMulticastListener<SSL_Referee>>(
          gamecontroller_multicast_address, gamecontroller_multicast_port,
          received_referee_callback))
{
}
