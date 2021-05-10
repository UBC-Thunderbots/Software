#include "software/simulation/simulator_robot_singleton.h"

#include "shared/proto/robot_log_msg.pb.h"
#include "software/logger/logger.h"
#include "software/world/field.h"

extern "C"
{
#include "firmware/app/logger/logger.h"
#include "firmware/app/world/charger.h"
#include "firmware/app/world/chicker.h"
#include "firmware/app/world/dribbler.h"
#include "firmware/app/world/force_wheel.h"
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
    checkValidAndExecuteVoid([firmware_world, primitive_msg](auto robot) {
        robot->startNewPrimitive(firmware_world, primitive_msg);
    });
}

void SimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(
    std::shared_ptr<FirmwareWorld_t> firmware_world)
{
    checkValidAndExecuteVoid(
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

void SimulatorRobotSingleton::checkValidAndExecuteVoid(
    std::function<void(std::shared_ptr<SimulatorRobot>)> func)
{
    if (simulator_robot)
    {
        func(simulator_robot);
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

float SimulatorRobotSingleton::checkValidAndReturnFloat(
    std::function<float(std::shared_ptr<SimulatorRobot>)> func)
{
    if (simulator_robot)
    {
        return func(simulator_robot);
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0.0f;
}

unsigned int SimulatorRobotSingleton::checkValidAndReturnUint(
    std::function<unsigned int(std::shared_ptr<SimulatorRobot>)> func)
{
    if (simulator_robot)
    {
        return func(simulator_robot);
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0;
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
    return checkValidAndReturnFloat(
        [](auto robot) { return invertValueToMatchFieldSide(robot->getPositionX()); });
}

float SimulatorRobotSingleton::getPositionY()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return invertValueToMatchFieldSide(robot->getPositionY()); });
}

float SimulatorRobotSingleton::getOrientation()
{
    return checkValidAndReturnFloat([](auto robot) {
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
    return checkValidAndReturnFloat(
        [](auto robot) { return invertValueToMatchFieldSide(robot->getVelocityX()); });
}

float SimulatorRobotSingleton::getVelocityY()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return invertValueToMatchFieldSide(robot->getVelocityY()); });
}

float SimulatorRobotSingleton::getVelocityAngular()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return robot->getVelocityAngular(); });
}

float SimulatorRobotSingleton::getBatteryVoltage()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return robot->getBatteryVoltage(); });
}

void SimulatorRobotSingleton::kick(float speed_m_per_s)
{
    checkValidAndExecuteVoid([speed_m_per_s](auto robot) { robot->kick(speed_m_per_s); });
}

void SimulatorRobotSingleton::chip(float distance_m)
{
    checkValidAndExecuteVoid([distance_m](auto robot) { robot->chip(distance_m); });
}

void SimulatorRobotSingleton::enableAutokick(float speed_m_per_s)
{
    checkValidAndExecuteVoid(
        [speed_m_per_s](auto robot) { robot->enableAutokick(speed_m_per_s); });
}

void SimulatorRobotSingleton::enableAutochip(float distance_m)
{
    checkValidAndExecuteVoid(
        [distance_m](auto robot) { robot->enableAutochip(distance_m); });
}

void SimulatorRobotSingleton::disableAutokick()
{
    checkValidAndExecuteVoid([](auto robot) { robot->disableAutokick(); });
}

void SimulatorRobotSingleton::disableAutochip()
{
    checkValidAndExecuteVoid([](auto robot) { robot->disableAutochip(); });
}

void SimulatorRobotSingleton::setDribblerSpeed(uint32_t rpm)
{
    checkValidAndExecuteVoid([rpm](auto robot) { robot->setDribblerSpeed(rpm); });
}

unsigned int SimulatorRobotSingleton::getDribblerTemperatureDegC()
{
    return checkValidAndReturnUint(
        [](auto robot) { return robot->getDribblerTemperatureDegC(); });
}

void SimulatorRobotSingleton::dribblerCoast()
{
    checkValidAndExecuteVoid([](auto robot) { robot->dribblerCoast(); });
}

float SimulatorRobotSingleton::getMotorSpeedFrontLeft()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return robot->getMotorSpeedFrontLeft(); });
}

float SimulatorRobotSingleton::getMotorSpeedBackLeft()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return robot->getMotorSpeedBackLeft(); });
}

float SimulatorRobotSingleton::getMotorSpeedBackRight()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return robot->getMotorSpeedBackRight(); });
}

float SimulatorRobotSingleton::getMotorSpeedFrontRight()
{
    return checkValidAndReturnFloat(
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
