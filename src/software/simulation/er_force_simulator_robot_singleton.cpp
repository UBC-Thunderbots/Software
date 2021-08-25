#include "software/simulation/er_force_simulator_robot_singleton.h"

#include "shared/proto/robot_log_msg.pb.h"

extern "C"
{
#include "firmware/app/logger/logger.h"
#include "firmware/app/world/charger.h"
#include "firmware/app/world/velocity_wheel.h"
}

FieldSide ErForceSimulatorRobotSingleton::field_side_ = FieldSide::NEG_X;

// TODO (#2066): The JERK_LIMIT is copied from firmware/main/control/control.h
// which we currently can't include directly because it relies on firmware IO.
// We should inject it as a robot or control param instead.
#define JERK_LIMIT 40.0f  //(m/s^3)
// TODO (#2066): The WHEEL_MOTOR_PHASE_RESISTANCE is copied from firmware/main/io/wheels.h
// which we currently can't include directly because it is in firmware IO.
// We should inject it as a robot or control param instead.
#define WHEEL_MOTOR_PHASE_RESISTANCE 1.2f  // ohms—EC45 datasheet

std::shared_ptr<ErForceSimulatorRobot>
    ErForceSimulatorRobotSingleton::er_force_simulator_robot = nullptr;

void ErForceSimulatorRobotSingleton::startNewPrimitiveOnCurrentSimulatorRobot(
    std::shared_ptr<FirmwareWorld_t> firmware_world,
    const TbotsProto_Primitive& primitive_msg)
{
    checkValidAndExecute<void>([firmware_world, primitive_msg](auto robot) {
        robot->startNewPrimitive(firmware_world, primitive_msg);
    });
}

void ErForceSimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(
    std::shared_ptr<FirmwareWorld_t> firmware_world)
{
    checkValidAndExecute<void>(
        [firmware_world](auto robot) { robot->runCurrentPrimitive(firmware_world); });
}

void ErForceSimulatorRobotSingleton::handleBlueRobotLogProto(TbotsProto_RobotLog log)
{
    ErForceSimulatorRobotSingleton::handleRobotLogProto(log, "BLUE");
}

void ErForceSimulatorRobotSingleton::handleYellowRobotLogProto(TbotsProto_RobotLog log)
{
    ErForceSimulatorRobotSingleton::handleRobotLogProto(log, "YELLOW");
}

float ErForceSimulatorRobotSingleton::invertValueToMatchFieldSide(float value)
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

float ErForceSimulatorRobotSingleton::getPositionX()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return invertValueToMatchFieldSide(robot->getPositionX()); });
}

float ErForceSimulatorRobotSingleton::getPositionY()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return invertValueToMatchFieldSide(robot->getPositionY()); });
}

float ErForceSimulatorRobotSingleton::getOrientation()
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

float ErForceSimulatorRobotSingleton::getVelocityX()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return invertValueToMatchFieldSide(robot->getVelocityX()); });
}

float ErForceSimulatorRobotSingleton::getVelocityY()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return invertValueToMatchFieldSide(robot->getVelocityY()); });
}

float ErForceSimulatorRobotSingleton::getVelocityAngular()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getVelocityAngular(); });
}

float ErForceSimulatorRobotSingleton::getBatteryVoltage()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getBatteryVoltage(); });
}

void ErForceSimulatorRobotSingleton::kick(float speed_m_per_s)
{
    checkValidAndExecute<void>(
        [speed_m_per_s](auto robot) { robot->kick(speed_m_per_s); });
}

void ErForceSimulatorRobotSingleton::chip(float distance_m)
{
    checkValidAndExecute<void>([distance_m](auto robot) { robot->chip(distance_m); });
}

void ErForceSimulatorRobotSingleton::enableAutokick(float speed_m_per_s)
{
    checkValidAndExecute<void>(
        [speed_m_per_s](auto robot) { robot->enableAutokick(speed_m_per_s); });
}

void ErForceSimulatorRobotSingleton::enableAutochip(float distance_m)
{
    checkValidAndExecute<void>(
        [distance_m](auto robot) { robot->enableAutochip(distance_m); });
}

void ErForceSimulatorRobotSingleton::disableAutokick()
{
    checkValidAndExecute<void>([](auto robot) { robot->disableAutokick(); });
}

void ErForceSimulatorRobotSingleton::disableAutochip()
{
    checkValidAndExecute<void>([](auto robot) { robot->disableAutochip(); });
}

void ErForceSimulatorRobotSingleton::setDribblerSpeed(uint32_t rpm)
{
    checkValidAndExecute<void>([rpm](auto robot) { robot->setDribblerSpeed(rpm); });
}

unsigned int ErForceSimulatorRobotSingleton::getDribblerTemperatureDegC()
{
    return checkValidAndExecute<unsigned int>(
        [](auto robot) { return robot->getDribblerTemperatureDegC(); });
}

void ErForceSimulatorRobotSingleton::dribblerCoast()
{
    checkValidAndExecute<void>([](auto robot) { robot->dribblerCoast(); });
}

float ErForceSimulatorRobotSingleton::getMotorSpeedFrontLeft()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getMotorSpeedFrontLeft(); });
}

float ErForceSimulatorRobotSingleton::getMotorSpeedBackLeft()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getMotorSpeedBackLeft(); });
}

float ErForceSimulatorRobotSingleton::getMotorSpeedBackRight()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getMotorSpeedBackRight(); });
}

float ErForceSimulatorRobotSingleton::getMotorSpeedFrontRight()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getMotorSpeedFrontRight(); });
}

void ErForceSimulatorRobotSingleton::coastMotorFrontLeft()
{
    checkValidAndExecute<void>([](auto robot) { robot->coastMotorFrontLeft(); });
}

void ErForceSimulatorRobotSingleton::coastMotorBackLeft()
{
    checkValidAndExecute<void>([](auto robot) { robot->coastMotorBackLeft(); });
}

void ErForceSimulatorRobotSingleton::coastMotorBackRight()
{
    checkValidAndExecute<void>([](auto robot) { robot->coastMotorBackRight(); });
}

void ErForceSimulatorRobotSingleton::coastMotorFrontRight()
{
    checkValidAndExecute<void>([](auto robot) { robot->coastMotorFrontRight(); });
}

void ErForceSimulatorRobotSingleton::brakeMotorFrontLeft()
{
    checkValidAndExecute<void>([](auto robot) { robot->brakeMotorFrontLeft(); });
}

void ErForceSimulatorRobotSingleton::brakeMotorBackLeft()
{
    checkValidAndExecute<void>([](auto robot) { robot->brakeMotorBackLeft(); });
}

void ErForceSimulatorRobotSingleton::brakeMotorBackRight()
{
    checkValidAndExecute<void>([](auto robot) { robot->brakeMotorBackRight(); });
}

void ErForceSimulatorRobotSingleton::brakeMotorFrontRight()
{
    checkValidAndExecute<void>([](auto robot) { robot->brakeMotorFrontRight(); });
}

void ErForceSimulatorRobotSingleton::handleRobotLogProto(TbotsProto_RobotLog log,
                                                         const std::string& team_colour)
{
    // TODO #1804: we should be sending the RobotLog to the SimulatorBackend
    LOG(INFO) << "[" << team_colour << " ROBOT " << log.robot_id << " "
              << TbotsProto::LogLevel_Name(
                     static_cast<TbotsProto::LogLevel>(log.log_level))
              << "]"
              << "[" << log.file_name << ":" << log.line_number << "]: " << log.log_msg;
}

void ErForceSimulatorRobotSingleton::setSimulatorRobot(
    std::shared_ptr<ErForceSimulatorRobot> robot, FieldSide field_side)
{
    er_force_simulator_robot = robot;
    field_side_              = field_side;
}

std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter>
ErForceSimulatorRobotSingleton::createFirmwareRobot()
{
    // Charger does nothing in sim
    Charger_t* charger = app_charger_create([]() {}, []() {}, []() {});

    Chicker_t* chicker = app_chicker_create(
        &(ErForceSimulatorRobotSingleton::kick), &(ErForceSimulatorRobotSingleton::chip),
        &(ErForceSimulatorRobotSingleton::enableAutokick),
        &(ErForceSimulatorRobotSingleton::enableAutochip),
        &(ErForceSimulatorRobotSingleton::disableAutokick),
        &(ErForceSimulatorRobotSingleton::disableAutochip));

    Dribbler_t* dribbler = app_dribbler_create(
        &(ErForceSimulatorRobotSingleton::setDribblerSpeed),
        &(ErForceSimulatorRobotSingleton::dribblerCoast),
        &(ErForceSimulatorRobotSingleton::getDribblerTemperatureDegC));

    VelocityWheelConstants_t wheel_constants;
    wheel_constants.wheel_rotations_per_motor_rotation  = GEAR_RATIO;
    wheel_constants.wheel_radius                        = WHEEL_RADIUS;
    wheel_constants.motor_max_voltage_before_wheel_slip = WHEEL_SLIP_VOLTAGE_LIMIT;
    wheel_constants.motor_back_emf_per_rpm              = RPM_TO_VOLT;
    wheel_constants.motor_phase_resistance              = WHEEL_MOTOR_PHASE_RESISTANCE;
    wheel_constants.motor_current_per_unit_torque       = CURRENT_PER_TORQUE;
    VelocityWheel_t* front_left_wheel                   = app_velocity_wheel_create(
        &(ErForceSimulatorRobotSingleton::setTargetRPMFrontLeft),
        &(ErForceSimulatorRobotSingleton::getMotorSpeedFrontLeft),
        &(ErForceSimulatorRobotSingleton::brakeMotorFrontLeft),
        &(ErForceSimulatorRobotSingleton::coastMotorFrontLeft), wheel_constants);
    VelocityWheel_t* front_right_wheel = app_velocity_wheel_create(
        &(ErForceSimulatorRobotSingleton::setTargetRPMFrontRight),
        &(ErForceSimulatorRobotSingleton::getMotorSpeedFrontRight),
        &(ErForceSimulatorRobotSingleton::brakeMotorFrontRight),
        &(ErForceSimulatorRobotSingleton::coastMotorFrontRight), wheel_constants);
    VelocityWheel_t* back_left_wheel = app_velocity_wheel_create(
        &(ErForceSimulatorRobotSingleton::setTargetRPMBackLeft),
        &(ErForceSimulatorRobotSingleton::getMotorSpeedBackLeft),
        &(ErForceSimulatorRobotSingleton::brakeMotorBackLeft),
        &(ErForceSimulatorRobotSingleton::coastMotorBackLeft), wheel_constants);
    VelocityWheel_t* back_right_wheel = app_velocity_wheel_create(
        &(ErForceSimulatorRobotSingleton::setTargetRPMBackRight),
        &(ErForceSimulatorRobotSingleton::getMotorSpeedBackRight),
        &(ErForceSimulatorRobotSingleton::brakeMotorBackRight),
        &(ErForceSimulatorRobotSingleton::coastMotorBackRight), wheel_constants);

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
    FirmwareRobot_t* firmware_robot = app_firmware_robot_velocity_wheels_create(
        charger, chicker, dribbler, &(ErForceSimulatorRobotSingleton::getPositionX),
        &(ErForceSimulatorRobotSingleton::getPositionY),
        &(ErForceSimulatorRobotSingleton::getOrientation),
        &(ErForceSimulatorRobotSingleton::getVelocityX),
        &(ErForceSimulatorRobotSingleton::getVelocityY),
        &(ErForceSimulatorRobotSingleton::getVelocityAngular),
        &(ErForceSimulatorRobotSingleton::getBatteryVoltage), front_right_wheel,
        front_left_wheel, back_right_wheel, back_left_wheel, controller_state,
        robot_constants);

    return std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter>(firmware_robot,
                                                                  FirmwareRobotDeleter());
}

void ErForceSimulatorRobotSingleton::setTargetRPMFrontLeft(float rpm)
{
    checkValidAndExecute<void>([rpm](auto robot) { robot->setTargetRPMFrontLeft(rpm); });
}

void ErForceSimulatorRobotSingleton::setTargetRPMBackLeft(float rpm)
{
    checkValidAndExecute<void>([rpm](auto robot) { robot->setTargetRPMBackLeft(rpm); });
}

void ErForceSimulatorRobotSingleton::setTargetRPMBackRight(float rpm)
{
    checkValidAndExecute<void>([rpm](auto robot) { robot->setTargetRPMBackRight(rpm); });
}

void ErForceSimulatorRobotSingleton::setTargetRPMFrontRight(float rpm)
{
    checkValidAndExecute<void>([rpm](auto robot) { robot->setTargetRPMFrontRight(rpm); });
}
