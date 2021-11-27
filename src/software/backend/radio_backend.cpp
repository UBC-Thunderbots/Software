#include "software/backend/radio_backend.h"

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/backend/radio/robot_status.h"
#include "software/constants.h"
#include "software/util/generic_factory/generic_factory.hpp"

RadioBackend::RadioBackend(std::shared_ptr<const BackendConfig> config)
    : ssl_communication_config(
          config->getRadioBackendConfig()->getSslCommunicationConfig()),
      ssl_proto_client(boost::bind(&Backend::receiveSSLWrapperPacket, this, _1),
                       boost::bind(&Backend::receiveSSLReferee, this, _1),
                       ssl_communication_config),
      radio_output(DEFAULT_RADIO_CONFIG,
                   boost::bind(&RadioBackend::receiveRobotStatus, this, _1))
{
}

void RadioBackend::onValueReceived(TbotsProto::PrimitiveSet primitives)
{
    radio_output.sendPrimitives(primitives);
}

void RadioBackend::onValueReceived(World world)
{
    // Send the world to the robots directly via radio
    radio_output.sendVisionPacket(world.friendlyTeam(), world.ball());
}

void RadioBackend::receiveRobotStatus(RadioRobotStatus robot_status)
{
    Backend::receiveRobotStatus(*convertRobotStatusToRobotStatusProto(robot_status));
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Backend, RadioBackend, BackendConfig> factory;
