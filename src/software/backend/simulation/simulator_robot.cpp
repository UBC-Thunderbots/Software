#include "software/backend/simulation/simulator_robot.h"
#include "shared/constants.h"
#include "software/logger/init.h"

SimulatorRobot::SimulatorRobot(std::weak_ptr<PhysicsRobot> physics_robot) : physics_robot(physics_robot), autokick_speed_m_per_s(std::nullopt), autochip_distance_m(std::nullopt), dribbler_rpm(0), ball_in_dribbler_area(nullptr){
    if(auto robot = this->physics_robot.lock()) {
        robot->registerChickerBallStartContactCallback([this](PhysicsRobot *robot, PhysicsBall *ball) {
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
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0;
}

float SimulatorRobot::getPositionX()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->position().x();
    }
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0.0;
}

float SimulatorRobot::getPositionY()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->position().y();
    }
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0.0;
}

Point SimulatorRobot::position() {
    return Point(getPositionX(), getPositionY());
}

float SimulatorRobot::getOrientation()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->orientation().toRadians();
    }
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0.0;
}

float SimulatorRobot::getVelocityX()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->velocity().x();
    }
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0.0;
}

float SimulatorRobot::getVelocityY()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->velocity().y();
    }
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0.0;
}

Vector SimulatorRobot::velocity() {
   return Vector(getVelocityX(), getVelocityY());
}

float SimulatorRobot::getVelocityAngular()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->angularVelocity().toRadians();
    }
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0.0;
}

float SimulatorRobot::getBatteryVoltage()
{
    // We currently have 4s batteries on the robot that charge up to a little over
    // 16V, so we use 16 here to approxiamte a fully-charged battery
    // TODO: Should max battery voltage be a constant / injected robot param?
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
    }else {
        LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
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
    }else {
        LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
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
    }else {
        LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    }
}

void SimulatorRobot::applyWheelForceBackLeft(float force_in_newtons)
{
    if (auto robot = physics_robot.lock())
    {
        robot->applyWheelForceBackLeft(force_in_newtons);
    }
    else {
        LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    }
}

void SimulatorRobot::applyWheelForceBackRight(float force_in_newtons)
{
    if (auto robot = physics_robot.lock())
    {
        robot->applyWheelForceBackRight(force_in_newtons);
    }
    else {
        LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    }
}

void SimulatorRobot::applyWheelForceFrontRight(float force_in_newtons)
{
    if (auto robot = physics_robot.lock())
    {
        robot->applyWheelForceFrontRight(force_in_newtons);
    }
    else {
        LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    }
}

float SimulatorRobot::getMotorSpeedFrontLeft()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->getMotorSpeedFrontLeft();
    }
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0.0;
}

float SimulatorRobot::getMotorSpeedBackLeft()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->getMotorSpeedBackLeft();
    }
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0.0;
}

float SimulatorRobot::getMotorSpeedBackRight()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->getMotorSpeedBackRight();
    }
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0.0;
}

float SimulatorRobot::getMotorSpeedFrontRight()
{
    if (auto robot = physics_robot.lock())
    {
        return robot->getMotorSpeedFrontRight();
    }
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0.0;
}

void SimulatorRobot::coastMotorFrontLeft() {
    // We coast by simply doing nothing and not applying wheel force
}

void SimulatorRobot::coastMotorBackLeft() {
    // We coast by simply doing nothing and not applying wheel force
}

void SimulatorRobot::coastMotorBackRight() {
    // We coast by simply doing nothing and not applying wheel force
}

void SimulatorRobot::coastMotorFrontRight() {
    // We coast by simply doing nothing and not applying wheel force
}

void SimulatorRobot::brakeMotorFrontLeft() {
    if (auto robot = physics_robot.lock())
    {
        robot->brakeMotorFrontLeft();
    }
    else {
        LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    }
}

void SimulatorRobot::brakeMotorBackLeft() {
    if (auto robot = physics_robot.lock())
    {
        robot->brakeMotorBackLeft();
    }
    else {
        LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    }
}

void SimulatorRobot::brakeMotorBackRight() {
    if (auto robot = physics_robot.lock())
    {
        robot->brakeMotorBackRight();
    }
    else {
        LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    }
}

void SimulatorRobot::brakeMotorFrontRight() {
    if (auto robot = physics_robot.lock())
    {
        robot->brakeMotorFrontRight();
    }
    else {
        LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    }
}

void SimulatorRobot::onChickerBallContact(PhysicsRobot *physics_robot, PhysicsBall *physics_ball) {
    if(autokick_speed_m_per_s) {
        Vector kick_vector = Vector::createFromAngle(physics_robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).orientation());
        kick_vector = kick_vector.normalize(autokick_speed_m_per_s.value());
        physics_ball->kick(kick_vector);
    }else if(autochip_distance_m) {
        Vector chip_vector = Vector::createFromAngle(physics_robot->getRobotWithTimestamp(Timestamp::fromSeconds(0)).orientation());
        chip_vector = chip_vector.normalize(autochip_distance_m.value());
        physics_ball->chip(chip_vector);
    }
}

void SimulatorRobot::onDribblerBallContact(PhysicsRobot *physics_robot, PhysicsBall *physics_ball) {
    if(dribbler_rpm > 0) {
        auto robot = physics_robot->getRobotWithTimestamp(Timestamp::fromSeconds(0));
        auto ball = physics_ball->getBallWithTimestamp(Timestamp::fromSeconds(0));

        // To dribble, we apply a force towards the center and back of the dribbling area, closest to the chicker.
        // We vary the magnitude of the foce by how far the ball is from this "dribbling point". This more-or-less
        // acts like a tiny gravity well that sucks the ball into place, except with more force the further away
        // the ball is

        Point dribble_point = robot.position() + Vector::createFromAngle(robot.orientation()).normalize(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS - PhysicsRobot::dribbler_depth);
        Vector dribble_force_vector = dribble_point - ball.position();
        // concvert to cm so we operate on a small scale
        double dist_from_dribble_point_cm = dribble_force_vector.length() * 100;
        // Combine a polynomial with a slightly offset linear function. This shifts the intercept
        // with the x-axis to a small positive x-value, so that there is a small region when the
        // ball is extremely close to the back of the dribbler area (and close to the chicker) where
        // a tiny amount of force will be applied away from the robot. This helps prevent us from
        // applying a force into the robot while the ball is touching it and creating a net force
        // that moves the robot.
        //
        // The constants in this equation have been tuned manually so that the dribbling scenarios
        // in the unit tests pass, which represent reasonable dribbling behavior.
        double polynomial_component = 0.1 * std::pow(dist_from_dribble_point_cm, 4);
        double linear_component = ((1.0 / 10.0) * (dist_from_dribble_point_cm - 0.5));
        double dribble_force_magnitude = polynomial_component + linear_component;
        dribble_force_magnitude = std::clamp<double>(dribble_force_magnitude, 0, dribble_force_magnitude);
        dribble_force_vector = dribble_force_vector.normalize(dribble_force_magnitude);

        physics_ball->applyForce(dribble_force_vector);

        std::cout << "applied force " << dribble_force_vector << std::endl;
    }
}

void SimulatorRobot::onDribblerBallStartContact(PhysicsRobot *physics_robot, PhysicsBall *physics_ball) {
    ball_in_dribbler_area = physics_ball;
}

void SimulatorRobot::onDribblerBallEndContact(PhysicsRobot *physics_robot, PhysicsBall *physics_ball) {
    ball_in_dribbler_area = nullptr;
}
