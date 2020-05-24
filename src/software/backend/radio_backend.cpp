#include "software/backend/radio_backend.h"

#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string RadioBackend::name = "radio";

// TODO change callback to bind
RadioBackend::RadioBackend()
    : network_input(Util::Constants::SSL_VISION_DEFAULT_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_ADDRESS,
                    Util::Constants::SSL_GAMECONTROLLER_MULTICAST_PORT,
                    boost::bind(&RadioBackend::receiveWorld, this, _1),
                    Util::DynamicParameters->getAIControlConfig()->getRefboxConfig(),
                    Util::DynamicParameters->getCameraConfig()),
      radio_output(DEFAULT_RADIO_CONFIG, [this](RobotStatus status) {
          Subject<RobotStatus>::sendValueToObservers(status);
      })
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
    // Convert robot_status to TBotsRobotMsg
    TbotsRobotMsg robot_msg = convertRobotStatusToTbotsRobotMsg(&robot_status);
    //TbotsRobotMsg robot_msg;
    //SensorMsg sensor_msg;
    //sensor_msg.set_tbots_robot_msg(robot_msg);
    //Subject<SensorMsg>::sendValueToObservers(sensor_msg);
}

// TODO unit test this
TbotsRobotMsg RadioBackend::convertRobotStatusToTbotsRobotMsg(RobotStatus* robot_status)
{
    TbotsRobotMsg robot_msg;
    BreakBeamStatus break_beam_msg;
    FirmwareStatus firmware_status_msg;
    DribblerStatus dribbler_status_msg;
    PowerStatus power_status_msg;
    TemperatureStatus temperature_status_msg;

    robot_msg.set_robot_id(robot_status->robot);

    // TODO how to convert ErrorCode convert from robot_messages to ErrorCode

    break_beam_msg.set_ball_in_beam(robot_status->ball_in_beam);
    // TODO msg is float but robot_status is double
    break_beam_msg.set_break_beam_reading(robot_status->break_beam_reading);
    robot_msg.set_break_beam_status(break_beam_msg);

    // Insufficient ChipperKickerStatus

    // Insufficient DriveUnits

    firmware_status_msg.set_fw_build_id(robot_status->fw_build_id);
    robot_msg.set_firmware_status(firmware_status_msg);

    dribbler_status_msg.set_dribbler_rpm(robot_status->dribbler_speed);
    robot_msg.set_dribbler_status(dribbler_status_msg);

    // Insufficient NetworkStatus

    power_status_msg.set_battery_voltage(robot_status->battery_voltage);
    power_status_msg.set_capacitor_voltage(robot_status->capacitor_voltage);
    robot_msg.set_power_status(power_status_msg);

    temperature_status_msg.set_dribbler_temperature(robot_status->dribbler_temperature);
    temperature_status_msg.set_board_temperature(robot_status->board_temperature);
    robot_msg.set_temperature_status(temperature_status_msg);

    return robot_msg;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Backend, RadioBackend> factory;
