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
      radio_output(DEFAULT_RADIO_CONFIG,
                   boost::bind(&RadioBackend::receiveRobotStatus, this, _1))
{
}

void RadioBackend::onValueReceived(ConstPrimitiveVectorPtr primitives_ptr)
{
    radio_output.sendPrimitives(*primitives_ptr);
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
    auto robot_msg                 = convertRobotStatusToTbotsRobotMsg(robot_status);
    TbotsRobotMsg* added_robot_msg = sensor_msg.add_tbots_robot_msg();
    added_robot_msg                = robot_msg.release();
    Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Backend, RadioBackend> factory;
