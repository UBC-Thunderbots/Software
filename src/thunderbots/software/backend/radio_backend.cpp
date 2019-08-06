#include "backend/radio_backend.h"

#include "util/constants.h"
#include "util/ros_messages.h"

RadioBackend::RadioBackend(ros::NodeHandle node_handle)
    : network_input(Util::Constants::SSL_VISION_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    boost::bind(&RadioBackend::receiveWorld, this, _1)),
      radio_output(DEFAULT_RADIO_CONFIG,
                   node_handle),
      radio_output_thread(
          boost::bind(&RadioBackend::continuouslyUpdatePrimitivesFromBuffer, this)),
      in_destructor(false)
{
}

void RadioBackend::continuouslyUpdatePrimitivesFromBuffer()
{
    do
    {
        in_destructor_mutex.unlock();

        receivePrimitives(Observer<PrimitiveVecPtr>::getMostRecentValueFromBuffer());

        in_destructor_mutex.lock();
    } while (!in_destructor);
}

void RadioBackend::setMostRecentlyReceivedWorld(Backend::World world)
{
    std::scoped_lock lock(most_recently_received_world_mutex);
    most_recently_received_world = world;
}

void RadioBackend::setMostRecentlyReceivedPrimitives(Backend::PrimitiveVecPtr primitives)
{
    std::scoped_lock lock(most_recently_received_primitives_mutex);
    most_recently_received_primitives = std::move(primitives);
}

void RadioBackend::receivePrimitives(Backend::PrimitiveVecPtr primitives)
{
    setMostRecentlyReceivedPrimitives(std::move(primitives));
    updateRadio();
}

void RadioBackend::receiveWorld(Backend::World world)
{
    setMostRecentlyReceivedWorld(world);
    sendValueToObservers(world);
    updateRadio();
}

void RadioBackend::updateRadio()
{
    std::scoped_lock lock(most_recently_received_world_mutex,
                          most_recently_received_primitives_mutex);

    // Update Radio if we have all the information we need
    if (most_recently_received_world && most_recently_received_primitives)
    {
        // TODO: we should stop passing ros msgs so we don't have to to this
        auto world =
            Util::ROSMessages::createWorldFromROSMessage(*most_recently_received_world);

        radio_output.sendPrimitives(*most_recently_received_primitives,
                                    world.friendlyTeam(), world.ball());
    }
}
