#include "software/backend/simulation/simulator_robot_singleton.h"

#include "software/logger/init.h"

extern "C"
{
#include "firmware/main/app/world/chicker.h"
#include "firmware/main/app/world/dribbler.h"
#include "firmware/main/app/world/wheel.h"
}

// TODO: The JERK_LIMIT is copied from firmware/main/control/control.h
// which we currently can't include directly because it relies on firmware IO.
// We should inject is as a robot or control param instead.
#define JERK_LIMIT 40.0f  //(m/s^3)
// TODO: The WHEEL_MOTOR_PHASE_RESISTANCE is copied from firmware/main/io/wheels.h
// which we currently can't include directly because it is in firmware IO.
// We should inject is as a robot or control param instead.
#define WHEEL_MOTOR_PHASE_RESISTANCE 1.2f  // ohms—EC45 datasheet

std::shared_ptr<SimulatorRobot> SimulatorRobotSingleton::simulator_robot = nullptr;

void SimulatorRobotSingleton::setSimulatorRobot(std::shared_ptr<SimulatorRobot> robot)
{
    simulator_robot = robot;
}

std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter>
SimulatorRobotSingleton::createFirmwareRobot()
{
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

    WheelConstants_t wheel_constants = {
        .wheel_rotations_per_motor_rotation  = GEAR_RATIO,
        .wheel_radius                        = WHEEL_RADIUS,
        .motor_max_voltage_before_wheel_slip = WHEEL_SLIP_VOLTAGE_LIMIT,
        .motor_back_emf_per_rpm              = RPM_TO_VOLT,
        .motor_phase_resistance              = WHEEL_MOTOR_PHASE_RESISTANCE,
        .motor_current_per_unit_torque       = CURRENT_PER_TORQUE};
    Wheel_t* front_left_wheel = app_wheel_create(
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
    FirmwareRobot_t* firmware_robot = app_firmware_robot_create(
        chicker, dribbler, &(SimulatorRobotSingleton::getPositionX),
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

float SimulatorRobotSingleton::getPositionX()
{
    if (simulator_robot)
    {
        return simulator_robot->getPositionX();
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0.0f;
}

float SimulatorRobotSingleton::getPositionY()
{
    if (simulator_robot)
    {
        return simulator_robot->getPositionY();
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0.0f;
}

float SimulatorRobotSingleton::getOrientation()
{
    if (simulator_robot)
    {
        return simulator_robot->getOrientation();
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0.0f;
}

float SimulatorRobotSingleton::getVelocityX()
{
    if (simulator_robot)
    {
        return simulator_robot->getVelocityX();
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0.0f;
}

float SimulatorRobotSingleton::getVelocityY()
{
    if (simulator_robot)
    {
        return simulator_robot->getVelocityY();
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0.0f;
}

float SimulatorRobotSingleton::getVelocityAngular()
{
    if (simulator_robot)
    {
        return simulator_robot->getVelocityAngular();
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0.0f;
}

float SimulatorRobotSingleton::getBatteryVoltage()
{
    if (simulator_robot)
    {
        return simulator_robot->getBatteryVoltage();
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0.0f;
}

void SimulatorRobotSingleton::kick(float speed_m_per_s)
{
    if (simulator_robot)
    {
        simulator_robot->kick(speed_m_per_s);
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::chip(float distance_m)
{
    if (simulator_robot)
    {
        simulator_robot->chip(distance_m);
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::enableAutokick(float speed_m_per_s)
{
    if (simulator_robot)
    {
        simulator_robot->enableAutokick(speed_m_per_s);
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::enableAutochip(float distance_m)
{
    if (simulator_robot)
    {
        simulator_robot->enableAutochip(distance_m);
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::disableAutokick()
{
    if (simulator_robot)
    {
        simulator_robot->disableAutokick();
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::disableAutochip()
{
    if (simulator_robot)
    {
        simulator_robot->disableAutochip();
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::setDribblerSpeed(uint32_t rpm)
{
    if (simulator_robot)
    {
        simulator_robot->setDribblerSpeed(rpm);
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

unsigned int SimulatorRobotSingleton::getDribblerTemperatureDegC()
{
    if (simulator_robot)
    {
        return simulator_robot->getDribblerTemperatureDegC();
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0;
}

void SimulatorRobotSingleton::dribblerCoast()
{
    if (simulator_robot)
    {
        simulator_robot->dribblerCoast();
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::applyWheelForceFrontLeft(float force_in_newtons)
{
    if (simulator_robot)
    {
        simulator_robot->applyWheelForceFrontLeft(force_in_newtons);
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::applyWheelForceBackLeft(float force_in_newtons)
{
    if (simulator_robot)
    {
        simulator_robot->applyWheelForceBackLeft(force_in_newtons);
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::applyWheelForceBackRight(float force_in_newtons)
{
    if (simulator_robot)
    {
        simulator_robot->applyWheelForceBackRight(force_in_newtons);
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::applyWheelForceFrontRight(float force_in_newtons)
{
    if (simulator_robot)
    {
        simulator_robot->applyWheelForceFrontRight(force_in_newtons);
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

float SimulatorRobotSingleton::getMotorSpeedFrontLeft()
{
    if (simulator_robot)
    {
        return simulator_robot->getMotorSpeedFrontLeft();
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0.0f;
}

float SimulatorRobotSingleton::getMotorSpeedBackLeft()
{
    if (simulator_robot)
    {
        return simulator_robot->getMotorSpeedBackLeft();
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0.0f;
}

float SimulatorRobotSingleton::getMotorSpeedBackRight()
{
    if (simulator_robot)
    {
        return simulator_robot->getMotorSpeedBackRight();
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0.0f;
}

float SimulatorRobotSingleton::getMotorSpeedFrontRight()
{
    if (simulator_robot)
    {
        return simulator_robot->getMotorSpeedFrontRight();
    }
    LOG(WARNING)
        << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
        << std::endl;
    return 0.0f;
}

void SimulatorRobotSingleton::coastMotorFrontLeft()
{
    if (simulator_robot)
    {
        simulator_robot->coastMotorFrontLeft();
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::coastMotorBackLeft()
{
    if (simulator_robot)
    {
        simulator_robot->coastMotorBackLeft();
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::coastMotorBackRight()
{
    if (simulator_robot)
    {
        simulator_robot->coastMotorBackRight();
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::coastMotorFrontRight()
{
    if (simulator_robot)
    {
        simulator_robot->coastMotorFrontRight();
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::brakeMotorFrontLeft()
{
    if (simulator_robot)
    {
        simulator_robot->brakeMotorFrontLeft();
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::brakeMotorBackLeft()
{
    if (simulator_robot)
    {
        simulator_robot->brakeMotorBackLeft();
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::brakeMotorBackRight()
{
    if (simulator_robot)
    {
        simulator_robot->brakeMotorBackRight();
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}

void SimulatorRobotSingleton::brakeMotorFrontRight()
{
    if (simulator_robot)
    {
        simulator_robot->brakeMotorFrontRight();
    }
    else
    {
        LOG(WARNING)
            << "SimulatorRobotSingleton called without setting the SimulatorRobot first"
            << std::endl;
    }
}
