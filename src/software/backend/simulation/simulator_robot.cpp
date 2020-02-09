#include "software/backend/simulation/simulator_robot.h"
#include "software/backend/simulation/physics/physics_ball.h"
#include "shared/constants.h"

SimulatorRobot::SimulatorRobot(std::weak_ptr<PhysicsRobot> physics_robot) : physics_robot(physics_robot), autokick_speed_m_per_s(std::nullopt), autochip_distance_m(std::nullopt), dribbler_rpm(0), ball_in_dribbler_area(nullptr){
    if(auto robot = this->physics_robot.lock()) {
        robot->registerChickerBallContactCallback([this](PhysicsRobot* robot, PhysicsBall* ball) {
            this->onChickerBallContact(robot, ball);
        });
        robot->registerDribblerBallContactCallback([this](PhysicsRobot* robot, PhysicsBall* ball) {
            this->onDribblerBallContact(robot, ball);
        });
        robot->registerDribblerBallStartContactCallback([this](PhysicsRobot* robot, PhysicsBall* ball) {
            this->onDribblerBallStartContact(robot, ball);
        });
        robot->registerDribblerBallEndContactCallback([this](PhysicsRobot* robot, PhysicsBall* ball) {
            this->onDribblerBallEndContact(robot, ball);
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

Point SimulatorRobot::position() {
    return Point(getPositionX(), getPositionY());
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

Vector SimulatorRobot::velocity() {
   return Vector(getVelocityX(), getVelocityY());
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
        if(ball_in_dribbler_area) {
            Vector kick_vector = Vector::createFromAngle(robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).orientation());
            kick_vector = kick_vector.normalize(speed_m_per_s);
            ball_in_dribbler_area->kick(kick_vector);
        }
    }
}

void SimulatorRobot::chip(float distance_m)
{
    if (auto robot = physics_robot.lock())
    {
        if(ball_in_dribbler_area) {
            Vector chip_vector = Vector::createFromAngle(robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).orientation());
            chip_vector = chip_vector.normalize(distance_m);
            ball_in_dribbler_area->chip(chip_vector);
        }
    }
}

void SimulatorRobot::enableAutokick(float speed_m_per_s)
{
    autokick_speed_m_per_s = speed_m_per_s;
}

void SimulatorRobot::enableAutochip(float distance_m)
{
    autochip_distance_m = distance_m;
}

void SimulatorRobot::disableAutokick()
{
    autokick_speed_m_per_s = std::nullopt;
}

void SimulatorRobot::disableAutochip()
{
    autochip_distance_m = std::nullopt;
}

void SimulatorRobot::setDribblerSpeed(uint32_t rpm)
{
    dribbler_rpm = rpm;
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
        robot->applyWheelForceFrontLeft(force_in_newtons);
    }
}

void SimulatorRobot::applyWheelForceBackLeft(float force_in_newtons)
{
    if (auto robot = physics_robot.lock())
    {
        robot->applyWheelForceBackLeft(force_in_newtons);
    }
}

void SimulatorRobot::applyWheelForceBackRight(float force_in_newtons)
{
    if (auto robot = physics_robot.lock())
    {
        robot->applyWheelForceBackRight(force_in_newtons);
    }
}

void SimulatorRobot::applyWheelForceFrontRight(float force_in_newtons)
{
    if (auto robot = physics_robot.lock())
    {
        robot->applyWheelForceFrontRight(force_in_newtons);
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


void SimulatorRobot::coastMotorFrontLeft() {
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::coastMotorBackLeft() {
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::coastMotorBackRight() {
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::coastMotorFrontRight() {
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::brakeMotorFrontLeft() {
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::brakeMotorBackLeft() {
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::brakeMotorBackRight() {
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::brakeMotorFrontRight() {
    if (auto robot = physics_robot.lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobot::onChickerBallContact(PhysicsRobot *robot, PhysicsBall *ball) {
    if(autokick_speed_m_per_s) {
        Vector kick_vector = Vector::createFromAngle(robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).orientation());
        kick_vector = kick_vector.normalize(autokick_speed_m_per_s.value());
        ball->kick(kick_vector);
    }else if(autochip_distance_m) {
        Vector chip_vector = Vector::createFromAngle(robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).orientation());
        chip_vector = chip_vector.normalize(autochip_distance_m.value());
        ball->chip(chip_vector);
    }
}

void SimulatorRobot::onDribblerBallContact(PhysicsRobot *robot, PhysicsBall *ball) {
//    std::cout << "dribbler ball contact callback" << std::endl;
    if(dribbler_rpm > 0) {
        // TODO: better bariable name
        auto r = robot->getRobotWithTimestamp(Timestamp::fromSeconds(0));
        // Solve v = rw,  where v = tangential velocity, r = radius, w = angular velocity
//        double dribbler_tangential_velocity_magnitude = ROBOT_MAX_RADIUS_METERS * r.angularVelocity().toRadians();
//        Vector dribbler_tangential_velocity = Vector::createFromAngle(r.orientation()).rotate(Angle::quarter());
//        dribbler_tangential_velocity = dribbler_tangential_velocity.normalize(dribbler_tangential_velocity_magnitude);
//        Vector dribbler_velocity = r.velocity() + dribbler_tangential_velocity;

        auto b = ball->getBallWithTimestamp(Timestamp::fromSeconds(0));
//        Vector velocity_diff = dribbler_velocity - b.velocity();

        // TODO: DONT" HARDCODE
        double chicker_depth = BALL_MAX_RADIUS_METERS * 2 * MAX_FRACTION_OF_BALL_COVERED_BY_ROBOT;
        Point dribble_point = r.position() + Vector::createFromAngle(r.orientation()).normalize(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS - chicker_depth);
        Vector force_vector = dribble_point - b.position();
        double dist = force_vector.length() * 100; // cm
//        double magnitude = 0.01 * std::pow(dist, 8);
        double magnitude = 0.1 * std::pow(dist, 4) + ((1.0/50.0)*(dist - 1));
//        double magnitude = force_vector.length();
        force_vector = force_vector.normalize(magnitude);

        ball->applyForce(force_vector);
        std::cout << "applied force " << force_vector << std::endl;
        // Pushed ball away when moving
//        ball->applyForce(velocity_diff);
//        Vector dribbler_force_vector = -Vector::createFromAngle(robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).orientation());
//        dribbler_force_vector = dribbler_force_vector.normalize(0.5);
//        ball->applyForce(dribbler_force_vector);
//        std::cout << "applied force " << dribbler_velocity << std::endl;
    }
}

void SimulatorRobot::onDribblerBallStartContact(PhysicsRobot *robot, PhysicsBall *ball) {
    ball_in_dribbler_area = ball;
}

void SimulatorRobot::onDribblerBallEndContact(PhysicsRobot *robot, PhysicsBall *ball) {
    ball_in_dribbler_area = nullptr;
}
