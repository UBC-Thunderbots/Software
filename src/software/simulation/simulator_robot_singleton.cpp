#include "software/simulation/simulator_robot_singleton.h"

#include "shared/proto/robot_log_msg.pb.h"

extern "C"
{
#include "firmware/app/logger/logger.h"
#include "firmware/app/world/charger.h"
}

std::shared_ptr<SimulatorRobot> SimulatorRobotSingleton::simulator_robot = nullptr;
FieldSide SimulatorRobotSingleton::field_side_ = FieldSide::NEG_X;

void SimulatorRobotSingleton::setSimulatorRobot(std::shared_ptr<SimulatorRobot> robot,
                                                FieldSide field_side)
{
    simulator_robot = robot;
    field_side_     = field_side;
}

void SimulatorRobotSingleton::startNewPrimitiveOnCurrentSimulatorRobot(
    std::shared_ptr<FirmwareWorld_t> firmware_world,
    const TbotsProto_Primitive& primitive_msg)
{
    checkValidAndExecute<void>([firmware_world, primitive_msg](auto robot) {
        robot->startNewPrimitive(firmware_world, primitive_msg);
    });
}

void SimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(
    std::shared_ptr<FirmwareWorld_t> firmware_world)
{
    checkValidAndExecute<void>(
        [firmware_world](auto robot) { robot->runCurrentPrimitive(firmware_world); });
}

void SimulatorRobotSingleton::handleBlueRobotLogProto(TbotsProto_RobotLog log)
{
    SimulatorRobotSingleton::handleRobotLogProto(log, "BLUE");
}

void SimulatorRobotSingleton::handleYellowRobotLogProto(TbotsProto_RobotLog log)
{
    SimulatorRobotSingleton::handleRobotLogProto(log, "YELLOW");
}

float SimulatorRobotSingleton::invertValueToMatchFieldSide(float value)
{
    switch (field_side_)
    {
        case FieldSide::NEG_X:
            return value;
        case FieldSide::POS_X:
            return -value;
        default:
            throw std::invalid_argument("Unhandled value of FieldSide");
    }
}

float SimulatorRobotSingleton::getPositionX()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return invertValueToMatchFieldSide(robot->getPositionX()); });
}

float SimulatorRobotSingleton::getPositionY()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return invertValueToMatchFieldSide(robot->getPositionY()); });
}

float SimulatorRobotSingleton::getOrientation()
{
    return checkValidAndExecute<float>([](auto robot) {
        switch (field_side_)
        {
            case FieldSide::NEG_X:
                return robot->getOrientation();
            case FieldSide::POS_X:
                return robot->getOrientation() + static_cast<float>(M_PI);
            default:
                throw std::invalid_argument("Unhandled value of FieldSide");
        }
    });
}

float SimulatorRobotSingleton::getVelocityX()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return invertValueToMatchFieldSide(robot->getVelocityX()); });
}

float SimulatorRobotSingleton::getVelocityY()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return invertValueToMatchFieldSide(robot->getVelocityY()); });
}

float SimulatorRobotSingleton::getVelocityAngular()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getVelocityAngular(); });
}

float SimulatorRobotSingleton::getBatteryVoltage()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getBatteryVoltage(); });
}

void SimulatorRobotSingleton::kick(float speed_m_per_s)
{
    checkValidAndExecute<void>(
        [speed_m_per_s](auto robot) { robot->kick(speed_m_per_s); });
}

void SimulatorRobotSingleton::chip(float distance_m)
{
    checkValidAndExecute<void>([distance_m](auto robot) { robot->chip(distance_m); });
}

void SimulatorRobotSingleton::enableAutokick(float speed_m_per_s)
{
    checkValidAndExecute<void>(
        [speed_m_per_s](auto robot) { robot->enableAutokick(speed_m_per_s); });
}

void SimulatorRobotSingleton::enableAutochip(float distance_m)
{
    checkValidAndExecute<void>(
        [distance_m](auto robot) { robot->enableAutochip(distance_m); });
}

void SimulatorRobotSingleton::disableAutokick()
{
    checkValidAndExecute<void>([](auto robot) { robot->disableAutokick(); });
}

void SimulatorRobotSingleton::disableAutochip()
{
    checkValidAndExecute<void>([](auto robot) { robot->disableAutochip(); });
}

void SimulatorRobotSingleton::setDribblerSpeed(uint32_t rpm)
{
    checkValidAndExecute<void>([rpm](auto robot) { robot->setDribblerSpeed(rpm); });
}

unsigned int SimulatorRobotSingleton::getDribblerTemperatureDegC()
{
    return checkValidAndExecute<unsigned int>(
        [](auto robot) { return robot->getDribblerTemperatureDegC(); });
}

void SimulatorRobotSingleton::dribblerCoast()
{
    checkValidAndExecute<void>([](auto robot) { robot->dribblerCoast(); });
}

float SimulatorRobotSingleton::getMotorSpeedFrontLeft()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getMotorSpeedFrontLeft(); });
}

float SimulatorRobotSingleton::getMotorSpeedBackLeft()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getMotorSpeedBackLeft(); });
}

float SimulatorRobotSingleton::getMotorSpeedBackRight()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getMotorSpeedBackRight(); });
}

float SimulatorRobotSingleton::getMotorSpeedFrontRight()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getMotorSpeedFrontRight(); });
}

void SimulatorRobotSingleton::handleRobotLogProto(TbotsProto_RobotLog log,
                                                  const std::string& team_colour)
{
    // TODO #1804: we should be sending the RobotLog to the WifiBackend
    LOG(INFO) << "[" << team_colour << " ROBOT " << log.robot_id << " "
              << TbotsProto::LogLevel_Name(
                     static_cast<TbotsProto::LogLevel>(log.log_level))
              << "]"
              << "[" << log.file_name << ":" << log.line_number << "]: " << log.log_msg;
}
