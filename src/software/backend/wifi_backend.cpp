#include "software/backend/wifi_backend.h"

#include "shared/constants.h"
#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/proto/message_translation/tbots_protobuf.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string WifiBackend::name = "wifi";

WifiBackend::WifiBackend()
    : network_input(Util::Constants::SSL_VISION_DEFAULT_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_PORT,
                    boost::bind(&WifiBackend::receiveWorld, this, _1),
                    Util::DynamicParameters->getAIControlConfig()->getRefboxConfig(),
                    Util::DynamicParameters->getCameraConfig())
{
    std::string network_interface =
        Util::DynamicParameters->getNetworkConfig()->NetworkInterface()->value();
    int channel = Util::DynamicParameters->getNetworkConfig()->Channel()->value();

    // connect to current channel
    joinMulticastChannel(channel, network_interface);
}

void WifiBackend::onValueReceived(ConstPrimitiveVectorPtr primitives_ptr)
{
    primitive_output->sendProto(*createPrimitiveMsg(primitives_ptr));
}

void WifiBackend::receiveWorld(World world)
{
    vision_output->sendProto(*createVisionMsg(world));
    Subject<World>::sendValueToObservers(world);
}

void WifiBackend::receiveTbotsRobotMsg(TbotsRobotMsg robot_msg)
{
    SensorMsg sensor_msg;
    TbotsRobotMsg* added_robot_msg = sensor_msg.add_tbots_robot_msg();
    *added_robot_msg               = robot_msg;
    Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}

void WifiBackend::joinMulticastChannel(int channel, const std::string& interface)
{
    vision_output.reset(new ThreadedProtoMulticastSender<VisionMsg>(
        std::string(MULTICAST_CHANNELS[channel]) + "%" + interface, VISION_PORT));

    primitive_output.reset(new ThreadedProtoMulticastSender<PrimitiveMsg>(
        std::string(MULTICAST_CHANNELS[channel]) + "%" + interface, PRIMITIVE_PORT));

    robot_msg_input.reset(new ThreadedProtoMulticastListener<TbotsRobotMsg>(
        std::string(MULTICAST_CHANNELS[channel]) + "%" + interface, ROBOT_STATUS_PORT,
        boost::bind(&WifiBackend::receiveTbotsRobotMsg, this, _1)));
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, WifiBackend> factory;
