#include "software/backend/simulation/simulator_robot.h"

#include "app/world/chicker.h"
#include "app/world/dribbler.h"
#include "app/world/wheel.h"

void SimulatorRobot::setRobotId(unsigned int id)
{
    SimulatorRobot::robot_id = id;
}

FirmwareRobot_t* SimulatorRobot::createFirmwareRobot()
{
    Chicker_t* chicker = app_chicker_create(
        &(SimulatorRobot::kick), &(SimulatorRobot::chip),
        &(SimulatorRobot::enableAutokick), &(SimulatorRobot::enableAutochip),
        &(SimulatorRobot::disableAutokick), &(SimulatorRobot::disableAutochip));

    Dribbler_t* dribbler =
        app_dribbler_create(&(SimulatorRobot::setDribblerSpeed),
                            &(SimulatorRobot::getDribblerTemperatureDegC));

    Wheel_t* front_left_wheel =
        app_wheel_create(&(SimulatorRobot::applyWheelForceFrontLeft));
    Wheel_t* front_right_wheel =
        app_wheel_create(&(SimulatorRobot::applyWheelForceFrontRight));
    Wheel_t* back_left_wheel =
        app_wheel_create(&(SimulatorRobot::applyWheelForceBackLeft));
    Wheel_t* back_right_wheel =
        app_wheel_create(&(SimulatorRobot::applyWheelForceBackRight));

    FirmwareRobot_t* firmware_robot =
        app_firmware_robot_create(chicker, dribbler, &(SimulatorRobot::getPositionX),
                                  &(SimulatorRobot::getPositionY), front_right_wheel,
                                  front_left_wheel, back_right_wheel, back_left_wheel);

    return firmware_robot;
}

float SimulatorRobot::getPositionX() {}

float SimulatorRobot::getPositionY() {}

void SimulatorRobot::kick(float speed_m_per_s) {}

void SimulatorRobot::chip(float distance_m) {}

void SimulatorRobot::enableAutokick(float speed_m_per_s) {}

void SimulatorRobot::enableAutochip(float distance_m) {}

void SimulatorRobot::disableAutokick() {}

void SimulatorRobot::disableAutochip() {}

void SimulatorRobot::setDribblerSpeed(uint32_t rpm) {}

unsigned int SimulatorRobot::getDribblerTemperatureDegC()
{
    // Return a somewhat arbitrary "room temperature" temperature.
    // This is an ideal simulation so the dribbler will not overheat
    return 25;
}

void SimulatorRobot::applyWheelForceFrontLeft(float force_in_newtons) {}

void SimulatorRobot::applyWheelForceBackLeft(float force_in_newtons) {}

void SimulatorRobot::applyWheelForceBackRight(float force_in_newtons) {}

void SimulatorRobot::applyWheelForceFrontRight(float force_in_newtons) {}
