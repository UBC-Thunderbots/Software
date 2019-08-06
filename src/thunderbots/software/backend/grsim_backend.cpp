#include "backend/grsim_backend.h"

#include "util/constants.h"
#include "util/ros_messages.h"

GrSimBackend::GrSimBackend()
    : network_input(Util::Constants::SSL_VISION_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    boost::bind(&GrSimBackend::receiveWorld, this, _1)),
      grsim_output(Util::Constants::GRSIM_COMMAND_NETWORK_ADDRESS,
                   Util::Constants::GRSIM_COMMAND_NETWORK_PORT),
      grsim_output_thread(
          boost::bind(&GrSimBackend::continuouslyUpdatePrimitivesFromBuffer, this)),
      in_destructor(false)
{
}

void GrSimBackend::continuouslyUpdatePrimitivesFromBuffer()
{
    do
    {
        in_destructor_mutex.unlock();

        receivePrimitives(Observer<PrimitiveVecPtr>::getMostRecentValueFromBuffer());

        in_destructor_mutex.lock();
    } while (!in_destructor);
}

void GrSimBackend::setMostRecentlyReceivedWorld(Backend::World world)
{
    std::scoped_lock lock(most_recently_received_world_mutex);
    most_recently_received_world = world;
}

void GrSimBackend::setMostRecentlyReceivedPrimitives(Backend::PrimitiveVecPtr primitives)
{
    std::scoped_lock lock(most_recently_received_primitives_mutex);
    most_recently_received_primitives = std::move(primitives);
}

void GrSimBackend::receivePrimitives(Backend::PrimitiveVecPtr primitives)
{
    setMostRecentlyReceivedPrimitives(std::move(primitives));
    updateGrSim();
}

void GrSimBackend::receiveWorld(Backend::World world)
{
    setMostRecentlyReceivedWorld(world);
    sendValueToObservers(world);
    updateGrSim();
}

void GrSimBackend::updateGrSim()
{
    std::scoped_lock lock(most_recently_received_world_mutex,
                          most_recently_received_primitives_mutex);

    // Update GrSim if we have all the information we need
    if (most_recently_received_world && most_recently_received_primitives)
    {
        // TODO: we should stop passing ros msgs so we don't have to to this
        auto world =
            Util::ROSMessages::createWorldFromROSMessage(*most_recently_received_world);

        grsim_output.sendPrimitives(*most_recently_received_primitives,
                                    world.friendlyTeam(), world.ball());
    }
}
