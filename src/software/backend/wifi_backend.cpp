#include "software/backend/radio_backend.h"
#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string WifiBackend::name = "radio";

WifiBackend::WifiBackend()
    : network_input(Util::Constants::SSL_VISION_DEFAULT_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_PORT,
                    boost::bind(&WifiBackend::receiveWorld, this, _1),
                    Util::DynamicParameters->getAIControlConfig()->getRefboxConfig(),
                    Util::DynamicParameters->getCameraConfig()),
      vision_output(
          Subject<RobotStatus>::sendValueToObservers(status);
})
{
}

void WifiBackend::onValueReceived(ConstPrimitiveVectorPtr primitives_ptr)
{
    primitives_ptr.sendPrimitives(*primitives_ptr);
}

void WifiBackend::receiveWorld(World world)
{
    vision_sender.sendProto(getVisionMsgFromWorld(world));

    Subject<World>::sendValueToObservers(world);
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, WifiBackend> factory;
