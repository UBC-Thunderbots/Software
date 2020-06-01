#include "software/backend/radio_backend.h"
#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"
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
    // connect to current channel
    connectOnChannel(Util::DynamicParameters->getNetworkConfig()->Channel()->value());

    // add callback to switch channels on param change
    Util::DynamicParameters->getNetworkConfig()->Channel()->registerCallbackFunction(
        boost::bind(&WifiBackend::connectOnChannel, this, _1));
}

void WifiBackend::onValueReceived(ConstPrimitiveVectorPtr primitives_ptr)
{
    primitive_output->sendProto(
        ProtobufMessageTranslator::getPrimitiveMsgFromPrimitiveVector(primitives_ptr));
}

void WifiBackend::receiveWorld(World world)
{
    vision_output->sendProto(ProtobufMessageTranslator::getVisionMsgFromWorld(world));
    Subject<World>::sendValueToObservers(world);
}

void WifiBackend::receiveTbotsRobotMsg(TbotsRobotMsg robot_msg)
{
    // TODO handle me
}

void WifiBackend::connectOnChannel(int channel)
{
    // when ProtoMulticastSenders and ProtoMulticastListeners are destroyed,
    // they close the sockets before freeing memory
    vision_output.reset(new ProtoMulticastSender<VisionMsg>(
        io_service, MULTICAST_CHANNELS[channel], VISION_PORT));

    primitive_output.reset(new ProtoMulticastSender<PrimitiveMsg>(
        io_service, MULTICAST_CHANNELS[channel], PRIMITIVE_PORT));

    robot_status_input.reset(new ProtoMulticastListener<TbotsRobotMsg>(
        io_service, MULTICAST_CHANNELS[channel], ROBOT_STATUS_PORT,
        boost::bind(&WifiBackend::receiveTbotsRobotMsg, this, _1));
    );
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, WifiBackend> factory;
