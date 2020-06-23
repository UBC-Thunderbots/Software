#include "software/backend/radio_backend.h"

#include "software/backend/robot_status.h"
#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string RadioBackend::name = "radio";

RadioBackend::RadioBackend()
    : network_input(Util::Constants::SSL_VISION_DEFAULT_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_PORT,
                    boost::bind(&RadioBackend::receiveWorld, this, _1),
                    Util::DynamicParameters->getAIControlConfig()->getRefboxConfig(),
                    Util::DynamicParameters->getCameraConfig()),
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
    SensorMsg sensor_msg;
    auto robot_msg                      = convertRobotStatusToTbotsRobotMsg(robot_status);
    *(sensor_msg.add_tbots_robot_msg()) = *robot_msg;
    Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Backend, RadioBackend> factory;
