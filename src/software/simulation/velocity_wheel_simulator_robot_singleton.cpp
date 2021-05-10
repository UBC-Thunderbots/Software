#include "software/simulation/velocity_wheel_simulator_robot_singleton.h"

#include "shared/proto/robot_log_msg.pb.h"

extern "C"
{
#include "firmware/app/logger/logger.h"
#include "firmware/app/world/velocity_wheel.h"
}

// TODO (#2066): The JERK_LIMIT is copied from firmware/main/control/control.h
// which we currently can't include directly because it relies on firmware IO.
// We should inject it as a robot or control param instead.
#define JERK_LIMIT 40.0f  //(m/s^3)
// TODO (#2066): The WHEEL_MOTOR_PHASE_RESISTANCE is copied from firmware/main/io/wheels.h
// which we currently can't include directly because it is in firmware IO.
// We should inject it as a robot or control param instead.
#define WHEEL_MOTOR_PHASE_RESISTANCE 1.2f  // ohms—EC45 datasheet

std::shared_ptr<VelocityWheelSimulatorRobot>
    VelocityWheelSimulatorRobotSingleton::velocity_wheel_simulator_robot = nullptr;

void VelocityWheelSimulatorRobotSingleton::setSimulatorRobot(
    std::shared_ptr<VelocityWheelSimulatorRobot> robot, FieldSide field_side)
{
    velocity_wheel_simulator_robot = robot;
    SimulatorRobotSingleton::setSimulatorRobot(robot, field_side);
}

std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter>
VelocityWheelSimulatorRobotSingleton::createFirmwareRobot()
{
    // Charger does nothing in sim
    Charger_t* charger = app_charger_create([]() {}, []() {}, []() {});

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

    VelocityWheelConstants_t wheel_constants;
    wheel_constants.wheel_rotations_per_motor_rotation  = GEAR_RATIO;
    wheel_constants.wheel_radius                        = WHEEL_RADIUS;
    wheel_constants.motor_max_voltage_before_wheel_slip = WHEEL_SLIP_VOLTAGE_LIMIT;
    wheel_constants.motor_back_emf_per_rpm              = RPM_TO_VOLT;
    wheel_constants.motor_phase_resistance              = WHEEL_MOTOR_PHASE_RESISTANCE;
    wheel_constants.motor_current_per_unit_torque       = CURRENT_PER_TORQUE;
    VelocityWheel_t* front_left_wheel                   = app_velocity_wheel_create(
        &(VelocityWheelSimulatorRobotSingleton::setTargetRPMFrontLeft),
        &(SimulatorRobotSingleton::getMotorSpeedFrontLeft),
        &(SimulatorRobotSingleton::brakeMotorFrontLeft),
        &(SimulatorRobotSingleton::coastMotorFrontLeft), wheel_constants);
    VelocityWheel_t* front_right_wheel = app_velocity_wheel_create(
        &(VelocityWheelSimulatorRobotSingleton::setTargetRPMFrontRight),
        &(SimulatorRobotSingleton::getMotorSpeedFrontRight),
        &(SimulatorRobotSingleton::brakeMotorFrontRight),
        &(SimulatorRobotSingleton::coastMotorFrontRight), wheel_constants);
    VelocityWheel_t* back_left_wheel = app_velocity_wheel_create(
        &(VelocityWheelSimulatorRobotSingleton::setTargetRPMBackLeft),
        &(SimulatorRobotSingleton::getMotorSpeedBackLeft),
        &(SimulatorRobotSingleton::brakeMotorBackLeft),
        &(SimulatorRobotSingleton::coastMotorBackLeft), wheel_constants);
    VelocityWheel_t* back_right_wheel = app_velocity_wheel_create(
        &(VelocityWheelSimulatorRobotSingleton::setTargetRPMBackRight),
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
    FirmwareRobot_t* firmware_robot = app_firmware_robot_velocity_wheels_create(
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

void VelocityWheelSimulatorRobotSingleton::setTargetRPMFrontLeft(float rpm)
{
    checkValidAndExecute<void>([rpm](auto robot) { robot->setTargetRPMFrontLeft(rpm); });
}

void VelocityWheelSimulatorRobotSingleton::setTargetRPMBackLeft(float rpm)
{
    checkValidAndExecute<void>([rpm](auto robot) { robot->setTargetRPMBackLeft(rpm); });
}

void VelocityWheelSimulatorRobotSingleton::setTargetRPMBackRight(float rpm)
{
    checkValidAndExecute<void>([rpm](auto robot) { robot->setTargetRPMBackRight(rpm); });
}

void VelocityWheelSimulatorRobotSingleton::setTargetRPMFrontRight(float rpm)
{
    checkValidAndExecute<void>([rpm](auto robot) { robot->setTargetRPMFrontRight(rpm); });
}
