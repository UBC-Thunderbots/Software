#include "software/backend/simulation/simulator_robot.h"

SimulatorRobot::SimulatorRobot(std::weak_ptr<PhysicsRobot> physics_robot) : physics_robot(physics_robot){
    // TODO: store id here instead of physics robot?
}

unsigned int SimulatorRobot::getRobotId() {
    // TODO: fix this
    if (auto robot = physics_robot.lock())
    {
        return robot->getRobotId();
    }
    return 0;
}

float SimulatorRobot::getPositionX()
{
    // Temporary implementation for testing
    if (auto robot = physics_robot.lock())
    {
        return static_cast<float>(robot->getRobotId());
    }
    return 0.0;
}

float SimulatorRobot::getPositionY()
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
    return 0.0;
}

float SimulatorRobot::getOrientation()
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
    return 0.0;
}

float SimulatorRobot::getVelocityX()
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
    return 0.0;
}

float SimulatorRobot::getVelocityY()
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
    return 0.0;
}

float SimulatorRobot::getVelocityAngular()
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
    return 0.0;
}

float SimulatorRobot::getBatteryVoltage()
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
    return 0.0;
}

void SimulatorRobot::kick(float speed_m_per_s)
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::chip(float distance_m)
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::enableAutokick(float speed_m_per_s)
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::enableAutochip(float distance_m)
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::disableAutokick()
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::disableAutochip()
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::setDribblerSpeed(uint32_t rpm)
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

unsigned int SimulatorRobot::getDribblerTemperatureDegC()
{
    // Return a somewhat arbitrary "room temperature" temperature.
    // This is an ideal simulation so the dribbler will not overheat
    return 25;
}

void SimulatorRobot::dribblerCoast()
{
    // Do nothing
}

void SimulatorRobot::applyWheelForceFrontLeft(float force_in_newtons)
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::applyWheelForceBackLeft(float force_in_newtons)
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::applyWheelForceBackRight(float force_in_newtons)
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::applyWheelForceFrontRight(float force_in_newtons)
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

float SimulatorRobot::getMotorSpeedFrontLeft()
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
    return 0.0;
}

float SimulatorRobot::getMotorSpeedBackLeft()
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
    return 0.0;
}

float SimulatorRobot::getMotorSpeedBackRight()
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
    return 0.0;
}

float SimulatorRobot::getMotorSpeedFrontRight()
{
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
    return 0.0;
}
