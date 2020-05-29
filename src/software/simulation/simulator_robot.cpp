#include "software/simulation/simulator_robot.h"

#include "shared/constants.h"
#include "software/logger/logger.h"

SimulatorRobot::SimulatorRobot(std::weak_ptr<PhysicsRobot> physics_robot)
    : physics_robot(physics_robot),
      autokick_speed_m_per_s(std::nullopt),
      autochip_distance_m(std::nullopt),
      dribbler_rpm(0)
{
    if (auto robot = this->physics_robot.lock())
    {
        robot->registerChickerBallStartContactCallback(
            [this](PhysicsRobot *robot, PhysicsBall *ball) {
                this->onChickerBallContact(robot, ball);
            });
        robot->registerDribblerBallContactCallback(
            [this](PhysicsRobot *robot, PhysicsBall *ball) {
                this->onDribblerBallContact(robot, ball);
            });
        robot->registerDribblerBallStartContactCallback(
            [this](PhysicsRobot *robot, PhysicsBall *ball) {
                this->onDribblerBallStartContact(robot, ball);
            });
        robot->registerDribblerBallEndContactCallback(
            [this](PhysicsRobot *robot, PhysicsBall *ball) {
                this->onDribblerBallEndContact(robot, ball);
            });
    }

    primitive_manager = std::unique_ptr<PrimitiveManager, PrimitiveManagerDeleter>(
        app_primitive_manager_create(), PrimitiveManagerDeleter());
}

void SimulatorRobot::checkValidAndExecuteVoid(
    std::function<void(std::shared_ptr<PhysicsRobot>)> func)
{
    if (auto robot = physics_robot.lock())
    {
        func(robot);
    }
    else
    {
        LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot"
                     << std::endl;
    }
}

float SimulatorRobot::checkValidAndReturnFloat(
    std::function<float(std::shared_ptr<PhysicsRobot>)> func)
{
    if (auto robot = physics_robot.lock())
    {
        return func(robot);
    }
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0.0f;
}

unsigned int SimulatorRobot::checkValidAndReturnUint(
    std::function<unsigned int(std::shared_ptr<PhysicsRobot>)> func)
{
    if (auto robot = physics_robot.lock())
    {
        return func(robot);
    }
    LOG(WARNING) << "SimulatorRobot being used with invalid PhysicsRobot" << std::endl;
    return 0;
}

unsigned int SimulatorRobot::getRobotId()
{
    return checkValidAndReturnUint([](auto robot) { return robot->getRobotId(); });
}

float SimulatorRobot::getPositionX()
{
    return checkValidAndReturnFloat([](auto robot) { return robot->position().x(); });
}

float SimulatorRobot::getPositionY()
{
    return checkValidAndReturnFloat([](auto robot) { return robot->position().y(); });
}

float SimulatorRobot::getOrientation()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return robot->orientation().toRadians(); });
}

float SimulatorRobot::getVelocityX()
{
    return checkValidAndReturnFloat([](auto robot) { return robot->velocity().x(); });
}

float SimulatorRobot::getVelocityY()
{
    return checkValidAndReturnFloat([](auto robot) { return robot->velocity().y(); });
}

float SimulatorRobot::getVelocityAngular()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return robot->angularVelocity().toRadians(); });
}

float SimulatorRobot::getBatteryVoltage()
{
    // We currently have 4s batteries on the robot that charge up to a little over
    // 16V, so we use 16 here to approximate a fully-charged battery
    // TODO: Should max battery voltage be a constant / injected robot param?
    // See https://github.com/UBC-Thunderbots/Software/issues/1173
    return 16.0;
}

void SimulatorRobot::kick(float speed_m_per_s)
{
    checkValidAndExecuteVoid([this, speed_m_per_s](auto robot) {
        for (auto ball : this->balls_in_dribbler_area)
        {
            Vector kick_vector =
                Vector::createFromAngle(robot->getRobotState().orientation());
            kick_vector = kick_vector.normalize(speed_m_per_s);
            ball->kick(kick_vector);
        }
    });
}

void SimulatorRobot::chip(float distance_m)
{
    checkValidAndExecuteVoid([this, distance_m](auto robot) {
        for (auto ball : this->balls_in_dribbler_area)
        {
            Vector chip_vector =
                Vector::createFromAngle(robot->getRobotState().orientation());
            chip_vector = chip_vector.normalize(distance_m);
            ball->chip(chip_vector);
        }
    });
}

void SimulatorRobot::enableAutokick(float speed_m_per_s)
{
    autokick_speed_m_per_s = speed_m_per_s;
    disableAutochip();
}

void SimulatorRobot::enableAutochip(float distance_m)
{
    autochip_distance_m = distance_m;
    disableAutokick();
}

void SimulatorRobot::disableAutokick()
{
    autokick_speed_m_per_s = std::nullopt;
}

void SimulatorRobot::disableAutochip()
{
    autochip_distance_m = std::nullopt;
}

bool SimulatorRobot::isAutokickEnabled()
{
    return autokick_speed_m_per_s.has_value();
}

bool SimulatorRobot::isAutochipEnabled()
{
    return autochip_distance_m.has_value();
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
    checkValidAndExecuteVoid([force_in_newtons](auto robot) {
        robot->applyWheelForceFrontLeft(force_in_newtons);
    });
}

void SimulatorRobot::applyWheelForceBackLeft(float force_in_newtons)
{
    checkValidAndExecuteVoid([force_in_newtons](auto robot) {
        robot->applyWheelForceBackLeft(force_in_newtons);
    });
}

void SimulatorRobot::applyWheelForceBackRight(float force_in_newtons)
{
    checkValidAndExecuteVoid([force_in_newtons](auto robot) {
        robot->applyWheelForceBackRight(force_in_newtons);
    });
}

void SimulatorRobot::applyWheelForceFrontRight(float force_in_newtons)
{
    checkValidAndExecuteVoid([force_in_newtons](auto robot) {
        robot->applyWheelForceFrontRight(force_in_newtons);
    });
}

float SimulatorRobot::getMotorSpeedFrontLeft()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return robot->getMotorSpeedFrontLeft(); });
}

float SimulatorRobot::getMotorSpeedBackLeft()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return robot->getMotorSpeedBackLeft(); });
}

float SimulatorRobot::getMotorSpeedBackRight()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return robot->getMotorSpeedBackRight(); });
}

float SimulatorRobot::getMotorSpeedFrontRight()
{
    return checkValidAndReturnFloat(
        [](auto robot) { return robot->getMotorSpeedFrontRight(); });
}

void SimulatorRobot::coastMotorFrontLeft()
{
    // We coast by simply doing nothing and not applying wheel force
}

void SimulatorRobot::coastMotorBackLeft()
{
    // We coast by simply doing nothing and not applying wheel force
}

void SimulatorRobot::coastMotorBackRight()
{
    // We coast by simply doing nothing and not applying wheel force
}

void SimulatorRobot::coastMotorFrontRight()
{
    // We coast by simply doing nothing and not applying wheel force
}

void SimulatorRobot::brakeMotorFrontLeft()
{
    checkValidAndExecuteVoid([](auto robot) { robot->brakeMotorFrontLeft(); });
}

void SimulatorRobot::brakeMotorBackLeft()
{
    checkValidAndExecuteVoid([](auto robot) { robot->brakeMotorBackLeft(); });
}

void SimulatorRobot::brakeMotorBackRight()
{
    checkValidAndExecuteVoid([](auto robot) { robot->brakeMotorBackRight(); });
}

void SimulatorRobot::brakeMotorFrontRight()
{
    checkValidAndExecuteVoid([](auto robot) { robot->brakeMotorFrontRight(); });
}

void SimulatorRobot::onChickerBallContact(PhysicsRobot *physics_robot,
                                          PhysicsBall *physics_ball)
{
    if (isAutokickEnabled())
    {
        Vector kick_vector =
            Vector::createFromAngle(physics_robot->getRobotState().orientation());
        kick_vector = kick_vector.normalize(autokick_speed_m_per_s.value());
        physics_ball->kick(kick_vector);
    }
    else if (isAutochipEnabled())
    {
        Vector chip_vector =
            Vector::createFromAngle(physics_robot->getRobotState().orientation());
        chip_vector = chip_vector.normalize(autochip_distance_m.value());
        physics_ball->chip(chip_vector);
    }
}

void SimulatorRobot::onDribblerBallContact(PhysicsRobot *physics_robot,
                                           PhysicsBall *physics_ball)
{
    if (dribbler_rpm > 0)
    {
        auto robot = physics_robot->getRobotState();
        auto ball  = physics_ball->getBallState();

        // To dribble, we apply a force towards the center and back of the dribbling area,
        // closest to the chicker. We vary the magnitude of the force by how far the ball
        // is from this "dribbling point". This more-or-less acts like a tiny gravity well
        // that sucks the ball into place, except with more force the further away the
        // ball is. Once the ball is no longer in the dribbler area this force is not
        // applied (it is only applied as long as the ball is in the dribbler area).

        Point dribble_point =
            robot.position() +
            Vector::createFromAngle(robot.orientation())
                .normalize(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS -
                           PhysicsRobot::dribbler_depth);
        Vector dribble_force_vector = dribble_point - ball.position();
        // convert to cm so we operate on a small scale
        double dist_from_dribble_point_cm =
            dribble_force_vector.length() * CENTIMETERS_PER_METER;
        // Combine a polynomial with a slightly offset linear function. This shifts the
        // intercept with the x-axis to a small positive x-value, so that there is a small
        // region when the ball is extremely close to the back of the dribbler area (and
        // close to the chicker) where a tiny amount of force will be applied away from
        // the robot. This helps prevent us from applying a force into the robot while the
        // ball is touching it and creating a net force that moves the robot.
        //
        // The constants in this equation have been tuned manually so that the dribbling
        // scenarios in the unit tests pass, which represent reasonable dribbling
        // behaviour.
        double polynomial_component = 0.1 * std::pow(dist_from_dribble_point_cm, 4);
        double linear_component     = ((1.0 / 10.0) * (dist_from_dribble_point_cm - 0.5));
        double dribble_force_magnitude = polynomial_component + linear_component;
        dribble_force_magnitude =
            std::clamp<double>(dribble_force_magnitude, 0, dribble_force_magnitude);
        dribble_force_vector = dribble_force_vector.normalize(dribble_force_magnitude);

        physics_ball->applyForce(dribble_force_vector);
    }
}

void SimulatorRobot::onDribblerBallStartContact(PhysicsRobot *physics_robot,
                                                PhysicsBall *physics_ball)
{
    balls_in_dribbler_area.emplace_back(physics_ball);
}

void SimulatorRobot::onDribblerBallEndContact(PhysicsRobot *physics_robot,
                                              PhysicsBall *physics_ball)
{
    auto iter = std::find(balls_in_dribbler_area.begin(), balls_in_dribbler_area.end(),
                          physics_ball);

    if (iter != balls_in_dribbler_area.end())
    {
        balls_in_dribbler_area.erase(iter);
    }
}

void SimulatorRobot::startNewPrimitive(std::shared_ptr<FirmwareWorld_t> firmware_world,
                                       unsigned int primitive_index,
                                       const primitive_params_t &params)
{
    app_primitive_manager_startNewPrimitive(primitive_manager.get(), firmware_world.get(),
                                            primitive_index, &params);
}

void SimulatorRobot::runCurrentPrimitive(std::shared_ptr<FirmwareWorld_t> firmware_world)
{
    app_primitive_manager_runCurrentPrimitive(primitive_manager.get(),
                                              firmware_world.get());
}
