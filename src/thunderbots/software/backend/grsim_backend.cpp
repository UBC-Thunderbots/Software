#include "backend/grsim_backend.h"

#include "util/constants.h"

GrSimBackend::GrSimBackend()
    : network_input(Util::Constants::SSL_VISION_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    boost::bind(&GrSimBackend::receiveWorld, this, _1)),
      grsim_output(Util::Constants::GRSIM_COMMAND_NETWORK_ADDRESS,
                   Util::Constants::GRSIM_COMMAND_NETWORK_PORT)
{
}

void GrSimBackend::onValueReceived(Backend::PrimitiveVecPtr primitives)
{
    setMostRecentlyReceivedPrimitives(std::move(primitives));
    updateGrSim();
}

void GrSimBackend::receiveWorld(World world)
{
    setMostRecentlyReceivedWorld(world);
    sendValueToObservers(world);
    updateGrSim();
}

void GrSimBackend::setMostRecentlyReceivedWorld(World world)
{
    std::scoped_lock lock(most_recently_received_world_mutex);
    most_recently_received_world = world;
}

void GrSimBackend::setMostRecentlyReceivedPrimitives(Backend::PrimitiveVecPtr primitives)
{
    std::scoped_lock lock(most_recently_received_primitives_mutex);
    most_recently_received_primitives = std::move(primitives);
}

void GrSimBackend::updateGrSim()
{
    std::scoped_lock lock(most_recently_received_world_mutex,
                          most_recently_received_primitives_mutex);

    // Update GrSim if we have all the information we need
    if (most_recently_received_world && most_recently_received_primitives)
    {
        grsim_output.sendPrimitives(*most_recently_received_primitives,
                                    most_recently_received_world->friendlyTeam(),
                                    most_recently_received_world->ball());
    }
}
