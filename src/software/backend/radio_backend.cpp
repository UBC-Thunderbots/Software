#include "software/backend/radio_backend.h"

#include "software/backend/robot_status.h"
#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string RadioBackend::name = "radio";

RadioBackend::RadioBackend(std::shared_ptr<const NetworkConfig> network_config)
    : network_config(network_config),
      network_input(network_config->VisionIP()->value(),
                    network_config->VisionPort()->value(),
                    network_config->GamecontrollerIP()->value(),
                    network_config->GamecontrollerPort()->value(),
                    boost::bind(&RadioBackend::receiveWorld, this, _1),
                    DynamicParameters->getSensorFusionConfig()),
      ssl_proto_client(boost::bind(&Backend::receiveSSLWrapperPacket, this, _1),
                       boost::bind(&Backend::receiveSSLReferee, this, _1),
                       network_config->VisionIP()->value(),
                       network_config->VisionPort()->value(),
                       network_config->GamecontrollerIP()->value(),
                       network_config->GamecontrollerPort()->value()),
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
