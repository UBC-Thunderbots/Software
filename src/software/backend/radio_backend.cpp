#include "software/backend/radio_backend.h"

#include "software/backend/robot_status.h"
#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string RadioBackend::name = "radio";

RadioBackend::RadioBackend()
    : network_input(SSL_VISION_DEFAULT_MULTICAST_ADDRESS, SSL_VISION_MULTICAST_PORT,
                    SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
                    SSL_GAMECONTROLLER_MULTICAST_PORT,
                    boost::bind(&RadioBackend::receiveWorld, this, _1),
                    DynamicParameters->getAIControlConfig()->getRefboxConfig(),
                    DynamicParameters->getCameraConfig()),
      ssl_proto_client(boost::bind(&Backend::receiveSSLWrapperPacket, this, _1),
                       boost::bind(&Backend::receiveSSLReferee, this, _1)),
      radio_output(DEFAULT_RADIO_CONFIG,
                   boost::bind(&RadioBackend::receiveRobotStatus, this, _1))
{
}

void RadioBackend::onValueReceived(ConstPrimitiveVectorPtr primitives_ptr)
{
    radio_output.sendPrimitives(*primitives_ptr);
}

void RadioBackend::onValueReceived(World world)
{
    // Send the world to the robots directly via radio
    radio_output.sendVisionPacket(world.friendlyTeam(), world.ball());
}

void RadioBackend::receiveWorld(World world)
{
    // Send the world to the robots directly via radio
    radio_output.sendVisionPacket(world.friendlyTeam(), world.ball());

    Subject<World>::sendValueToObservers(world);
}

void RadioBackend::receiveRobotStatus(RobotStatus robot_status)
{
    Backend::receiveTbotsRobotMsg(*convertRobotStatusToTbotsRobotMsg(robot_status));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Backend, RadioBackend> factory;
