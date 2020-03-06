#include "software/backend/grsim_backend.h"

#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string GrSimBackend::name = "grsim";

GrSimBackend::GrSimBackend()
    : network_input(Util::Constants::SSL_VISION_DEFAULT_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_PORT,
                    boost::bind(&GrSimBackend::receiveWorld, this, _1),
                    Util::DynamicParameters->getAIControlConfig()->getRefboxConfig(),
                    Util::DynamicParameters->getCameraConfig()),
      grsim_output(Util::Constants::GRSIM_COMMAND_NETWORK_ADDRESS,
                   Util::Constants::GRSIM_COMMAND_NETWORK_PORT,
                   Util::DynamicParameters->getAIControlConfig()->getRefboxConfig())
{
}

void GrSimBackend::onValueReceived(ConstPrimitiveVectorPtr primitives)
{
    setMostRecentlyReceivedPrimitives(primitives);
    updateGrSim();
}

void GrSimBackend::receiveWorld(World world)
{
    setMostRecentlyReceivedWorld(world);
    Subject<World>::sendValueToObservers(world);
    updateGrSim();
}

void GrSimBackend::setMostRecentlyReceivedWorld(World world)
{
    std::scoped_lock lock(most_recently_received_world_mutex);
    most_recently_received_world = world;
}

void GrSimBackend::setMostRecentlyReceivedPrimitives(ConstPrimitiveVectorPtr primitives)
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

// Register this play in the genericFactory
static TGenericFactory<std::string, Backend, GrSimBackend> factory;
