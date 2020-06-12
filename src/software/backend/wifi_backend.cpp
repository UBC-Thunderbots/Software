#include "software/backend/wifi_backend.h"

#include "shared/constants.h"
#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/proto/message_translation/protobuf_message_translation.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string WifiBackend::name = "wifi";

WifiBackend::WifiBackend()
    : network_input(Util::Constants::SSL_VISION_DEFAULT_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_PORT,
                    boost::bind(&WifiBackend::receiveWorld, this, _1),
                    Util::DynamicParameters->getAIControlConfig()->getRefboxConfig(),
                    Util::DynamicParameters->getCameraConfig()),
      io_service()
{
    std::string network_interface =
        Util::DynamicParameters->getNetworkConfig()->NetworkInterface()->value();
    int channel = Util::DynamicParameters->getNetworkConfig()->Channel()->value();

    // connect to current channel
    joinMulticastChannel(channel, network_interface);

    // start the thread to run the io_service in the background
    io_service_thread = std::thread([this]() { io_service.run(); });
}

WifiBackend::~WifiBackend()
{
    io_service_thread.join();
}

void WifiBackend::onValueReceived(ConstPrimitiveVectorPtr primitives_ptr)
{
    primitive_output->sendProto(
        *convertPrimitiveVectortoPrimitiveMsgProto(primitives_ptr));
}

void WifiBackend::receiveWorld(World world)
{
    vision_output->sendProto(*convertWorldToVisionMsgProto(world));
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
    vision_output.reset(new ProtoMulticastSender<VisionMsg>(
        io_service, std::string(MULTICAST_CHANNELS[channel]) + "%" + interface,
        VISION_PORT));

    primitive_output.reset(new ProtoMulticastSender<PrimitiveMsg>(
        io_service, std::string(MULTICAST_CHANNELS[channel]) + "%" + interface,
        PRIMITIVE_PORT));

    robot_msg_input.reset(new ProtoMulticastListener<TbotsRobotMsg>(
        io_service, std::string(MULTICAST_CHANNELS[channel]) + "%" + interface,
        ROBOT_STATUS_PORT, boost::bind(&WifiBackend::receiveTbotsRobotMsg, this, _1)));
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, WifiBackend> factory;
