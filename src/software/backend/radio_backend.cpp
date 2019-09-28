#include "software/backend/radio_backend.h"

#include "software/backend/backend_factory.h"
#include "software/util/constants.h"

const std::string RadioBackend::name = "radio";

RadioBackend::RadioBackend()
    : network_input(Util::Constants::SSL_VISION_DEFAULT_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_PORT,
                    boost::bind(&RadioBackend::receiveWorld, this, _1)),
      radio_output(DEFAULT_RADIO_CONFIG, [this](RobotStatus status) {
          Subject<RobotStatus>::sendValueToObservers(status);
      })
{
}

void RadioBackend::onValueReceived(ConstPrimitiveVectorPtr primitives_ptr)
{
    radio_output.sendPrimitives(*primitives_ptr);
}

void RadioBackend::receiveWorld(World world)
{
    // Send the world to the robots directly via radio
    radio_output.sendVisionPacket(world.friendlyTeam(), world.ball());

    Subject<World>::sendValueToObservers(world);
}

// Register this backend in the BackendFactory
static TBackendFactory<RadioBackend> factory;
