#include "software/backend/simulation/simulator_robot_singleton.h"

extern "C"
{
#include "app/world/chicker.h"
#include "app/world/dribbler.h"
#include "app/world/wheel.h"
}

std::shared_ptr<SimulatorRobot> SimulatorRobotSingleton::simulator_robot = nullptr;

// TODO: should have a null check on all functions in case this is not called first
void SimulatorRobotSingleton::setSimulatorRobot(std::shared_ptr<SimulatorRobot> robot) {
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
        .motor_phase_resistance              = PHASE_RESISTANCE,
        .motor_current_per_unit_torque       = CURRENT_PER_TORQUE};
    Wheel_t* front_left_wheel = app_wheel_create(
        &(SimulatorRobotSingleton::applyWheelForceFrontLeft),
        &(SimulatorRobotSingleton::getMotorSpeedFrontLeft), wheel_constants);
    Wheel_t* front_right_wheel = app_wheel_create(
        &(SimulatorRobotSingleton::applyWheelForceFrontRight),
        &(SimulatorRobotSingleton::getMotorSpeedFrontRight), wheel_constants);
    Wheel_t* back_left_wheel = app_wheel_create(
        &(SimulatorRobotSingleton::applyWheelForceBackLeft),
        &(SimulatorRobotSingleton::getMotorSpeedBackLeft), wheel_constants);
    Wheel_t* back_right_wheel = app_wheel_create(
        &(SimulatorRobotSingleton::applyWheelForceBackRight),
        &(SimulatorRobotSingleton::getMotorSpeedBackRight), wheel_constants);

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
    return simulator_robot->getPositionX();
}

float SimulatorRobotSingleton::getPositionY()
{
    return simulator_robot->getPositionY();
}

float SimulatorRobotSingleton::getOrientation()
{
    return simulator_robot->getOrientation();
}

float SimulatorRobotSingleton::getVelocityX()
{
    return simulator_robot->getVelocityX();
}

float SimulatorRobotSingleton::getVelocityY()
{
    return simulator_robot->getVelocityY();
}

float SimulatorRobotSingleton::getVelocityAngular()
{
    return simulator_robot->getVelocityAngular();
}

float SimulatorRobotSingleton::getBatteryVoltage()
{
    return simulator_robot->getBatteryVoltage();
}

void SimulatorRobotSingleton::kick(float speed_m_per_s)
{
    simulator_robot->kick(speed_m_per_s);
}

void SimulatorRobotSingleton::chip(float distance_m)
{
    simulator_robot->chip(distance_m);
}

void SimulatorRobotSingleton::enableAutokick(float speed_m_per_s)
{
    simulator_robot->enableAutokick(speed_m_per_s);
}

void SimulatorRobotSingleton::enableAutochip(float distance_m)
{
    simulator_robot->enableAutochip(distance_m);
}

void SimulatorRobotSingleton::disableAutokick()
{
    simulator_robot->disableAutokick();
}

void SimulatorRobotSingleton::disableAutochip()
{
    simulator_robot->disableAutochip();
}

void SimulatorRobotSingleton::setDribblerSpeed(uint32_t rpm)
{
    simulator_robot->setDribblerSpeed(rpm);
}

unsigned int SimulatorRobotSingleton::getDribblerTemperatureDegC()
{
    return simulator_robot->getDribblerTemperatureDegC();
}

void SimulatorRobotSingleton::dribblerCoast()
{
    simulator_robot->dribblerCoast();
}

void SimulatorRobotSingleton::applyWheelForceFrontLeft(float force_in_newtons)
{
    simulator_robot->applyWheelForceFrontLeft(force_in_newtons);
}

void SimulatorRobotSingleton::applyWheelForceBackLeft(float force_in_newtons)
{
    simulator_robot->applyWheelForceBackLeft(force_in_newtons);
}

void SimulatorRobotSingleton::applyWheelForceBackRight(float force_in_newtons)
{
    simulator_robot->applyWheelForceBackRight(force_in_newtons);
}

void SimulatorRobotSingleton::applyWheelForceFrontRight(float force_in_newtons)
{
    simulator_robot->applyWheelForceFrontRight(force_in_newtons);
}

float SimulatorRobotSingleton::getMotorSpeedFrontLeft()
{
    return simulator_robot->getMotorSpeedFrontLeft();
}

float SimulatorRobotSingleton::getMotorSpeedBackLeft()
{
    return simulator_robot->getMotorSpeedBackLeft();
}

float SimulatorRobotSingleton::getMotorSpeedBackRight()
{
    return simulator_robot->getMotorSpeedBackRight();
}

float SimulatorRobotSingleton::getMotorSpeedFrontRight()
{
    return simulator_robot->getMotorSpeedFrontRight();
}
