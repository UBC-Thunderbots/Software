#include "backend/radio_backend.h"

#include "util/constants.h"
#include "util/ros_messages.h"

RadioBackend::RadioBackend(ros::NodeHandle node_handle)
    : network_input(Util::Constants::SSL_VISION_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    boost::bind(&RadioBackend::receiveWorld, this, _1)),
      radio_output(DEFAULT_RADIO_CONFIG,
                   node_handle)
{
}

void RadioBackend::newValueCallback(Backend::PrimitiveVecPtr primitives_ptr){
    radio_output.sendPrimitives(*primitives_ptr);
}

void RadioBackend::receiveWorld(World world)
{
    radio_output.send_vision_packet(world.friendlyTeam(), world.ball());

    sendValueToObservers(world);
}
