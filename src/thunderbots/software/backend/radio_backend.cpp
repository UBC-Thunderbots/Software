#include "backend/radio_backend.h"

#include "backend/backend_factory.h"
#include "util/constants.h"

const std::string RadioBackend::name = "RadioBackend";

RadioBackend::RadioBackend()
    : network_input(Util::Constants::SSL_VISION_DEFAULT_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    boost::bind(&RadioBackend::receiveWorld, this, _1)),
      radio_output(DEFAULT_RADIO_CONFIG, [this](RobotStatus status) {
          Subject<RobotStatus>::sendValueToObservers(status);
      })
{
}

void RadioBackend::onValueReceived(Backend::PrimitiveVecPtr primitives_ptr)
{
    radio_output.sendPrimitives(*primitives_ptr);
}

void RadioBackend::receiveWorld(World world)
{
    // Send the world to the robots directly via radio
    radio_output.send_vision_packet(world.friendlyTeam(), world.ball());

    Subject<World>::sendValueToObservers(world);
}

// Register this backed in the BackendFactory
static TBackendFactory<RadioBackend> factory;
