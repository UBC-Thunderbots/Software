#include "software/backend/wifi_backend.h"

#include "shared/constants.h"
#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/proto/message_translation/tbots_protobuf.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string WifiBackend::name = "wifi";

WifiBackend::WifiBackend(std::shared_ptr<const NetworkConfig> network_config)
    : network_config(network_config),
      ssl_proto_client(boost::bind(&Backend::receiveSSLWrapperPacket, this, _1),
                       boost::bind(&Backend::receiveSSLReferee, this, _1),
                       network_config->getSSLCommunicationConfig())
{
    std::string network_interface =
        DynamicParameters->getNetworkConfig()->NetworkInterface()->value();
    int channel = DynamicParameters->getNetworkConfig()->Channel()->value();

    MutableDynamicParameters->getMutableNetworkConfig()
        ->mutableChannel()
        ->registerCallbackFunction([this](int new_channel) {
            std::string new_network_interface =
                DynamicParameters->getNetworkConfig()->NetworkInterface()->value();
            joinMulticastChannel(new_channel, new_network_interface);
        });

    // connect to current channel
    joinMulticastChannel(channel, network_interface);
}

void WifiBackend::onValueReceived(ConstPrimitiveVectorPtr primitives_ptr)
{
    primitive_output->sendProto(*createPrimitiveSetMsg(primitives_ptr));
}

void WifiBackend::onValueReceived(World world)
{
    vision_output->sendProto(*createVisionMsg(world));
}

void WifiBackend::joinMulticastChannel(int channel, const std::string& interface)
{
    vision_output.reset(new ThreadedProtoMulticastSender<VisionMsg>(
        std::string(MULTICAST_CHANNELS[channel]) + "%" + interface, VISION_PORT));

    primitive_output.reset(new ThreadedProtoMulticastSender<PrimitiveSetMsg>(
        std::string(MULTICAST_CHANNELS[channel]) + "%" + interface, PRIMITIVE_PORT));

    robot_msg_input.reset(new ThreadedProtoMulticastListener<TbotsRobotMsg>(
        std::string(MULTICAST_CHANNELS[channel]) + "%" + interface, ROBOT_STATUS_PORT,
        boost::bind(&Backend::receiveTbotsRobotMsg, this, _1)));
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, WifiBackend> factory;
