#include "software/backend/simulation/simulator_robot.h"
#include "software/backend/simulation/physics/physics_ball.h"

SimulatorRobot::SimulatorRobot(std::weak_ptr<PhysicsRobot> physics_robot) : physics_robot(physics_robot), kick_speed_m_per_s(std::nullopt), chip_distance_m(std::nullopt), dribbler_rpm(0){
    if(auto robot = this->physics_robot.lock()) {
        robot->registerChickerBallContactCallback([this](PhysicsRobot* robot, PhysicsBall* ball) {
            this->onChickerBallContact(robot, ball);
        });
        robot->registerDribblerBallContactCallback([this](PhysicsRobot* robot, PhysicsBall* ball) {
            this->onDribblerBallContact(robot, ball);
        });
    }
}

unsigned int SimulatorRobot::getRobotId() {
    if (auto robot = physics_robot.lock())
    {
        return robot->getRobotId();
    }
    return 0;
}

float SimulatorRobot::getPositionX()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).position().x();
    }
    return 0.0;
}

float SimulatorRobot::getPositionY()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).position().y();
    }
    return 0.0;
}

float SimulatorRobot::getOrientation()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).orientation().toRadians();
    }
    return 0.0;
}

float SimulatorRobot::getVelocityX()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).velocity().x();
    }
    return 0.0;
}

float SimulatorRobot::getVelocityY()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).velocity().y();
    }
    return 0.0;
}

float SimulatorRobot::getVelocityAngular()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).angularVelocity().toRadians();
    }
    return 0.0;
}

float SimulatorRobot::getBatteryVoltage()
{
    // TODO: comment this value
    return 16.0;
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
    kick_speed_m_per_s = speed_m_per_s;
}

void SimulatorRobot::enableAutochip(float distance_m)
{
    chip_distance_m = distance_m;
}

std::optional<double> SimulatorRobot::getAutokickSpeed() const {
    return kick_speed_m_per_s;
}

std::optional<double> SimulatorRobot::getAutochipDistance() const {
    return chip_distance_m;
}

void SimulatorRobot::disableAutokick()
{
    kick_speed_m_per_s = std::nullopt;
}

void SimulatorRobot::disableAutochip()
{
    chip_distance_m = std::nullopt;
}

void SimulatorRobot::setDribblerSpeed(uint32_t rpm)
{
    dribbler_rpm = rpm;
}

uint32_t SimulatorRobot::getDribblerSpeed() const {
    return dribbler_rpm;
}

unsigned int SimulatorRobot::getDribblerTemperatureDegC()
{
    // Return a somewhat arbitrary "room temperature" temperature.
    // This is an ideal simulation so the dribbler will not overheat
    return 25;
}

void SimulatorRobot::dribblerCoast()
{
    setDribblerSpeed(0);
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

void SimulatorRobot::onChickerBallContact(PhysicsRobot *robot, PhysicsBall *ball) {
    if(kick_speed_m_per_s) {
        Vector kick_vector = Vector::createFromAngle(robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).orientation());
        kick_vector = kick_vector.normalize(kick_speed_m_per_s.value());
        ball->kick(kick_vector);
    }else if(chip_distance_m) {
        Vector chip_vector = Vector::createFromAngle(robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).orientation());
        chip_vector = chip_vector.normalize(chip_distance_m.value());
        ball->chip(chip_vector);
    }
}

void SimulatorRobot::onDribblerBallContact(PhysicsRobot *robot, PhysicsBall *ball) {
    std::cout << "dribbler ball contact callback" << std::endl;
    if(dribbler_rpm > 0) {
        Vector dribbler_force_vector = -Vector::createFromAngle(robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).orientation());
        dribbler_force_vector = dribbler_force_vector.normalize(10000);
        ball->applyForce(dribbler_force_vector);
        std::cout << "applied force " << dribbler_force_vector << std::endl;
    }
}
