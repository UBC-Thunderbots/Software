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
#include "firmware/app/world/wheel.h"
}

// TODO: The JERK_LIMIT is copied from firmware/main/control/control.h
// which we currently can't include directly because it relies on firmware IO.
// We should inject it as a robot or control param instead.
#define JERK_LIMIT 40.0f  //(m/s^3)
// TODO: The WHEEL_MOTOR_PHASE_RESISTANCE is copied from firmware/main/io/wheels.h
// which we currently can't include directly because it is in firmware IO.
// We should inject it as a robot or control param instead.
#define WHEEL_MOTOR_PHASE_RESISTANCE 1.2f  // ohms—EC45 datasheet

std::shared_ptr<SimulatorRobot> SimulatorRobotSingleton::simulator_robot = nullptr;
FieldSide SimulatorRobotSingleton::field_side_ = FieldSide::NEG_X;

void SimulatorRobotSingleton::setSimulatorRobot(std::shared_ptr<SimulatorRobot> robot,
                                                FieldSide field_side)
{
    simulator_robot = robot;
    field_side_     = field_side;
}

std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter>
SimulatorRobotSingleton::createFirmwareRobot()
{
    // Charger does nothing in sim
    Charger_t* charger = app_charger_create([]() {}, []() {}, []() {});

    // TODO: Make sure all objects de-allocated properly
    // See issue https://github.com/UBC-Thunderbots/Software/issues/1128
    Chicker_t* chicker = app_chicker_create(&(SimulatorRobotSingleton::kick),
                                            &(SimulatorRobotSingleton::chip),
                                            &(SimulatorRobotSingleton::enableAutokick),
                                            &(SimulatorRobotSingleton::enableAutochip),
                                            &(SimulatorRobotSingleton::disableAutokick),
                                            &(SimulatorRobotSingleton::disableAutochip));

    Dribbler_t* dribbler =
        app_dribbler_create(&(SimulatorRobotSingleton::setDribblerSpeed),
                            &(SimulatorRobotSingleton::dribblerCoast),
                            &(SimulatorRobotSingleton::getDribblerTemperatureDegC));

    WheelConstants_t wheel_constants;
    wheel_constants.wheel_rotations_per_motor_rotation  = GEAR_RATIO;
    wheel_constants.wheel_radius                        = WHEEL_RADIUS;
    wheel_constants.motor_max_voltage_before_wheel_slip = WHEEL_SLIP_VOLTAGE_LIMIT;
    wheel_constants.motor_back_emf_per_rpm              = RPM_TO_VOLT;
    wheel_constants.motor_phase_resistance              = WHEEL_MOTOR_PHASE_RESISTANCE;
    wheel_constants.motor_current_per_unit_torque       = CURRENT_PER_TORQUE;
    Wheel_t* front_left_wheel                           = app_wheel_create(
        &(SimulatorRobotSingleton::applyWheelForceFrontLeft),
        &(SimulatorRobotSingleton::getMotorSpeedFrontLeft),
        &(SimulatorRobotSingleton::brakeMotorFrontLeft),
        &(SimulatorRobotSingleton::coastMotorFrontLeft), wheel_constants);
    Wheel_t* front_right_wheel = app_wheel_create(
        &(SimulatorRobotSingleton::applyWheelForceFrontRight),
        &(SimulatorRobotSingleton::getMotorSpeedFrontRight),
        &(SimulatorRobotSingleton::brakeMotorFrontRight),
        &(SimulatorRobotSingleton::coastMotorFrontRight), wheel_constants);
    Wheel_t* back_left_wheel =
        app_wheel_create(&(SimulatorRobotSingleton::applyWheelForceBackLeft),
                         &(SimulatorRobotSingleton::getMotorSpeedBackLeft),
                         &(SimulatorRobotSingleton::brakeMotorBackLeft),
                         &(SimulatorRobotSingleton::coastMotorBackLeft), wheel_constants);
    Wheel_t* back_right_wheel = app_wheel_create(
        &(SimulatorRobotSingleton::applyWheelForceBackRight),
        &(SimulatorRobotSingleton::getMotorSpeedBackRight),
        &(SimulatorRobotSingleton::brakeMotorBackRight),
        &(SimulatorRobotSingleton::coastMotorBackRight), wheel_constants);

    const RobotConstants_t robot_constants = {
        .mass              = ROBOT_POINT_MASS,
        .moment_of_inertia = INERTIA,
        .robot_radius      = ROBOT_RADIUS,
        .jerk_limit        = JERK_LIMIT,
    };
    ControllerState_t* controller_state = new ControllerState_t{
        .last_applied_acceleration_x       = 0,
        .last_applied_acceleration_y       = 0,
        .last_applied_acceleration_angular = 0,
    };
    FirmwareRobot_t* firmware_robot = app_firmware_wheels_robot_create(
        charger, chicker, dribbler, &(SimulatorRobotSingleton::getPositionX),
        &(SimulatorRobotSingleton::getPositionY),
        &(SimulatorRobotSingleton::getOrientation),
        &(SimulatorRobotSingleton::getVelocityX),
        &(SimulatorRobotSingleton::getVelocityY),
        &(SimulatorRobotSingleton::getVelocityAngular),
        &(SimulatorRobotSingleton::getBatteryVoltage), front_right_wheel,
        front_left_wheel, back_right_wheel, back_left_wheel, controller_state,
        robot_constants);

    return std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter>(firmware_robot,
                                                                  FirmwareRobotDeleter());
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

void SimulatorRobotSingleton::applyWheelForceFrontLeft(float force_in_newtons)
{
    checkValidAndExecuteVoid([force_in_newtons](auto robot) {
        robot->applyWheelForceFrontLeft(force_in_newtons);
    });
}

void SimulatorRobotSingleton::applyWheelForceBackLeft(float force_in_newtons)
{
    checkValidAndExecuteVoid([force_in_newtons](auto robot) {
        robot->applyWheelForceBackLeft(force_in_newtons);
    });
}

void SimulatorRobotSingleton::applyWheelForceBackRight(float force_in_newtons)
{
    checkValidAndExecuteVoid([force_in_newtons](auto robot) {
        robot->applyWheelForceBackRight(force_in_newtons);
    });
}

void SimulatorRobotSingleton::applyWheelForceFrontRight(float force_in_newtons)
{
    checkValidAndExecuteVoid([force_in_newtons](auto robot) {
        robot->applyWheelForceFrontRight(force_in_newtons);
    });
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

void SimulatorRobotSingleton::coastMotorFrontLeft()
{
    checkValidAndExecuteVoid([](auto robot) { robot->coastMotorFrontLeft(); });
}

void SimulatorRobotSingleton::coastMotorBackLeft()
{
    checkValidAndExecuteVoid([](auto robot) { robot->coastMotorBackLeft(); });
}

void SimulatorRobotSingleton::coastMotorBackRight()
{
    checkValidAndExecuteVoid([](auto robot) { robot->coastMotorBackRight(); });
}

void SimulatorRobotSingleton::coastMotorFrontRight()
{
    checkValidAndExecuteVoid([](auto robot) { robot->coastMotorFrontRight(); });
}

void SimulatorRobotSingleton::brakeMotorFrontLeft()
{
    checkValidAndExecuteVoid([](auto robot) { robot->brakeMotorFrontLeft(); });
}

void SimulatorRobotSingleton::brakeMotorBackLeft()
{
    checkValidAndExecuteVoid([](auto robot) { robot->brakeMotorBackLeft(); });
}

void SimulatorRobotSingleton::brakeMotorBackRight()
{
    checkValidAndExecuteVoid([](auto robot) { robot->brakeMotorBackRight(); });
}

void SimulatorRobotSingleton::brakeMotorFrontRight()
{
    checkValidAndExecuteVoid([](auto robot) { robot->brakeMotorFrontRight(); });
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
