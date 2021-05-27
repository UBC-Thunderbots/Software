#include "software/simulation/physics_simulator_robot.h"

#include "shared/constants.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/logger/logger.h"
#include "software/math/math_functions.h"

PhysicsSimulatorRobot::PhysicsSimulatorRobot(std::weak_ptr<PhysicsRobot> physics_robot)
    : physics_robot(physics_robot), dribbler_rpm(0)
{
    if (auto robot = this->physics_robot.lock())
    {
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
}

unsigned int PhysicsSimulatorRobot::getRobotId()
{
    return checkValidAndExecute<unsigned int>(
        [](auto robot) { return robot->getRobotId(); });
}

float PhysicsSimulatorRobot::getPositionX()
{
    return checkValidAndExecute<float>([](auto robot) { return robot->position().x(); });
}

float PhysicsSimulatorRobot::getPositionY()
{
    return checkValidAndExecute<float>([](auto robot) { return robot->position().y(); });
}

float PhysicsSimulatorRobot::getOrientation()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->orientation().toRadians(); });
}

float PhysicsSimulatorRobot::getVelocityX()
{
    return checkValidAndExecute<float>([](auto robot) { return robot->velocity().x(); });
}

float PhysicsSimulatorRobot::getVelocityY()
{
    return checkValidAndExecute<float>([](auto robot) { return robot->velocity().y(); });
}

float PhysicsSimulatorRobot::getVelocityAngular()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->angularVelocity().toRadians(); });
}

float PhysicsSimulatorRobot::getBatteryVoltage()
{
    return ROBOT_MAX_BATTERY_VOLTAGE;
}

void PhysicsSimulatorRobot::kick(float speed_m_per_s)
{
    checkValidAndExecute<void>([this, speed_m_per_s](auto robot) {
        if (ball_in_dribbler_area && ball_in_dribbler_area->can_be_controlled)
        {
            auto ball = ball_in_dribbler_area->ball;
            Vector robot_orientation_vector =
                Vector::createFromAngle(robot->getRobotState().orientation());

            double total_head_on_collision_momentum_magnitude = 0.0;

            if (acuteAngle(ball->velocity(), -robot_orientation_vector).abs() <
                Angle::quarter())
            {
                // The ball is heading towards the robot, and will have momentum that
                // is preserved in the kicking collision
                total_head_on_collision_momentum_magnitude +=
                    ball->momentum().project(robot_orientation_vector).length();
            }

            if (acuteAngle(robot->velocity(), robot_orientation_vector).abs() <
                Angle::quarter())
            {
                // The robot is moving in direction it's facing, and will add momentum
                // that is preserved in the kicking collision
                double robot_speed_in_direction_of_orientation =
                    robot->velocity().project(robot_orientation_vector).length();
                total_head_on_collision_momentum_magnitude +=
                    ball->massKg() * robot_speed_in_direction_of_orientation;
            }

            double preserved_momentum = total_head_on_collision_momentum_magnitude *
                                        MOMENTUM_CONSERVED_DURING_KICK;
            Vector kick_momentum =
                robot_orientation_vector.normalize(ball->massKg() * speed_m_per_s);
            Vector kick_impulse =
                kick_momentum.normalize(kick_momentum.length() + preserved_momentum);

            // Cancel head-on momentum, then kick. We must cancel the head-on momentum
            // first so that the preserved momentum has an additive effect that results in
            // a higher kick speed than requested.
            Vector ball_head_on_momentum =
                ball->momentum().project(robot_orientation_vector);
            // TODO (#1798): This is a hack that makes kicking while the dribbler on
            // slower
            ball->applyImpulse(robot_orientation_vector.normalize(
                ball_head_on_momentum.length() *
                (1 - (dribbler_rpm / MAX_FORCE_DRIBBLER_SPEED) / 2)));
            ball->applyImpulse(kick_impulse);
            ball->setInitialKickSpeed(speed_m_per_s);

            ball_in_dribbler_area->can_be_controlled = false;
        }
    });
}

void PhysicsSimulatorRobot::chip(float distance_m)
{
    checkValidAndExecute<void>([this, distance_m](auto robot) {
        if (ball_in_dribbler_area && ball_in_dribbler_area->can_be_controlled)
        {
            auto ball        = ball_in_dribbler_area->ball;
            Angle chip_angle = Angle::fromDegrees(ROBOT_CHIP_ANGLE_DEGREES);

            // Mark the ball as "in flight" so collisions are turned off
            // until it has travelled the desired chip distance.
            ball->setInFlightForDistance(distance_m, chip_angle);

            // Use the formula for the Range of a parabolic projectile
            // Rearrange to solve for the initial velocity.
            // https://courses.lumenlearning.com/boundless-physics/chapter/projectile-motion/
            float range = distance_m;
            float numerator =
                range *
                static_cast<float>(ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED);
            float denominator = static_cast<float>(2.0f * (chip_angle * 2.0f).sin());
            float initial_velocity =
                static_cast<float>(std::sqrt(numerator / denominator));
            float ground_velocity =
                initial_velocity * static_cast<float>(chip_angle.cos());
            kick(ground_velocity);

            ball_in_dribbler_area->can_be_controlled = false;
        }
    });
}

void PhysicsSimulatorRobot::enableAutokick(float speed_m_per_s)
{
    autokick_speed_m_per_s = speed_m_per_s;
    disableAutochip();
    // Kick any balls already in the dribbler
    kick(speed_m_per_s);
}

void PhysicsSimulatorRobot::enableAutochip(float distance_m)
{
    autochip_distance_m = distance_m;
    disableAutokick();
    // Chip any balls already in the dribbler
    chip(distance_m);
}

void PhysicsSimulatorRobot::disableAutokick()
{
    autokick_speed_m_per_s = std::nullopt;
}

void PhysicsSimulatorRobot::disableAutochip()
{
    autochip_distance_m = std::nullopt;
}

bool PhysicsSimulatorRobot::isAutokickEnabled()
{
    return autokick_speed_m_per_s.has_value();
}

bool PhysicsSimulatorRobot::isAutochipEnabled()
{
    return autochip_distance_m.has_value();
}

void PhysicsSimulatorRobot::setDribblerSpeed(uint32_t rpm)
{
    dribbler_rpm = rpm;
}

unsigned int PhysicsSimulatorRobot::getDribblerTemperatureDegC()
{
    // Return a somewhat arbitrary "room temperature" temperature.
    // This is an ideal simulation so the dribbler will not overheat
    return 25;
}

void PhysicsSimulatorRobot::dribblerCoast()
{
    setDribblerSpeed(0);
}

void PhysicsSimulatorRobot::applyWheelForceFrontLeft(float force_in_newtons)
{
    checkValidAndExecute<void>([force_in_newtons](auto robot) {
        robot->applyWheelForceFrontLeft(force_in_newtons);
    });
}

void PhysicsSimulatorRobot::applyWheelForceBackLeft(float force_in_newtons)
{
    checkValidAndExecute<void>([force_in_newtons](auto robot) {
        robot->applyWheelForceBackLeft(force_in_newtons);
    });
}

void PhysicsSimulatorRobot::applyWheelForceBackRight(float force_in_newtons)
{
    checkValidAndExecute<void>([force_in_newtons](auto robot) {
        robot->applyWheelForceBackRight(force_in_newtons);
    });
}

void PhysicsSimulatorRobot::applyWheelForceFrontRight(float force_in_newtons)
{
    checkValidAndExecute<void>([force_in_newtons](auto robot) {
        robot->applyWheelForceFrontRight(force_in_newtons);
    });
}

float PhysicsSimulatorRobot::getMotorSpeedFrontLeft()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getMotorSpeedFrontLeft(); });
}

float PhysicsSimulatorRobot::getMotorSpeedBackLeft()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getMotorSpeedBackLeft(); });
}

float PhysicsSimulatorRobot::getMotorSpeedBackRight()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getMotorSpeedBackRight(); });
}

float PhysicsSimulatorRobot::getMotorSpeedFrontRight()
{
    return checkValidAndExecute<float>(
        [](auto robot) { return robot->getMotorSpeedFrontRight(); });
}

void PhysicsSimulatorRobot::coastMotorFrontLeft()
{
    // We coast by simply doing nothing and not applying wheel force
}

void PhysicsSimulatorRobot::coastMotorBackLeft()
{
    // We coast by simply doing nothing and not applying wheel force
}

void PhysicsSimulatorRobot::coastMotorBackRight()
{
    // We coast by simply doing nothing and not applying wheel force
}

void PhysicsSimulatorRobot::coastMotorFrontRight()
{
    // We coast by simply doing nothing and not applying wheel force
}

void PhysicsSimulatorRobot::brakeMotorFrontLeft()
{
    checkValidAndExecute<void>([](auto robot) { robot->brakeMotorFrontLeft(); });
}

void PhysicsSimulatorRobot::brakeMotorBackLeft()
{
    checkValidAndExecute<void>([](auto robot) { robot->brakeMotorBackLeft(); });
}

void PhysicsSimulatorRobot::brakeMotorBackRight()
{
    checkValidAndExecute<void>([](auto robot) { robot->brakeMotorBackRight(); });
}

void PhysicsSimulatorRobot::brakeMotorFrontRight()
{
    checkValidAndExecute<void>([](auto robot) { robot->brakeMotorFrontRight(); });
}

void PhysicsSimulatorRobot::onDribblerBallContact(PhysicsRobot *physics_robot,
                                                  PhysicsBall *physics_ball)
{
    if (dribbler_rpm > 0 && ball_in_dribbler_area)
    {
        if (ball_in_dribbler_area->ball != physics_ball)
        {
            throw std::runtime_error("Trying to dribble ball not in the dribbler area");
        }

        if (ball_in_dribbler_area->can_be_controlled)
        {
            applyDribblerForce(physics_robot, physics_ball);
        }
    }
}

void PhysicsSimulatorRobot::onDribblerBallStartContact(PhysicsRobot *physics_robot,
                                                       PhysicsBall *physics_ball)
{
    // Damp the ball when it collides with the dribbler. We damp each component
    // of the ball's momentum separately so we have the flexibility to tune this
    // behavior to match real life.
    Vector ball_momentum       = physics_ball->momentum();
    Vector robot_facing_vector = Vector::createFromAngle(physics_robot->orientation());
    Vector robot_perp_vector   = robot_facing_vector.perpendicular();

    Vector dribbler_head_on_momentum = ball_momentum.project(robot_facing_vector);
    physics_ball->applyImpulse(-dribbler_head_on_momentum * DRIBBLER_HEAD_ON_DAMPING);

    Vector dribbler_perp_momentum = ball_momentum.project(robot_perp_vector);
    physics_ball->applyImpulse(-dribbler_perp_momentum * DRIBBLER_PERPENDICULAR_DAMPING);

    auto ball = DribblerBall{.ball = physics_ball, .can_be_controlled = true};

    // Keep track of all balls in the dribbler
    ball_in_dribbler_area = ball;

    // Even if the dribbler is on, we are guaranteed to apply kicking force
    // and disable ball control before the dribbler checks to apply dribbling force.
    // (So that the kick doesn't get messed up by dribbling).
    // This is because the "StartContact" callbacks are called before the
    // "PreSolve" contacts in the contact listener
    // 1.     onDribblerBallStartContactCallback is called
    // 2.     damping force is applied
    // 3.     autokick is triggered if it's on
    // 4.     the onDribblerBallContact callback is called
    // 5.     Dribbling force would be applied here if the ball was not kicked
    if (isAutokickEnabled())
    {
        kick(autokick_speed_m_per_s.value());
    }
    else if (isAutochipEnabled())
    {
        chip(autochip_distance_m.value());
    }
}

void PhysicsSimulatorRobot::onDribblerBallEndContact(PhysicsRobot *physics_robot,
                                                     PhysicsBall *physics_ball)
{
    clearBallInDribblerArea();
}

void PhysicsSimulatorRobot::startNewPrimitive(
    std::shared_ptr<FirmwareWorld_t> firmware_world,
    const TbotsProto_Primitive &primitive_msg)
{
    app_primitive_manager_startNewPrimitive(primitive_manager.get(), firmware_world.get(),
                                            primitive_msg);
}

void PhysicsSimulatorRobot::runCurrentPrimitive(
    std::shared_ptr<FirmwareWorld_t> firmware_world)
{
    app_primitive_manager_runCurrentPrimitive(primitive_manager.get(),
                                              firmware_world.get());
}

void PhysicsSimulatorRobot::clearBallInDribblerArea()
{
    ball_in_dribbler_area = std::nullopt;
}

void PhysicsSimulatorRobot::applyDribblerForce(PhysicsRobot *physics_robot,
                                               PhysicsBall *physics_ball)
{
    auto robot = physics_robot->getRobotState();
    auto ball  = physics_ball->getBallState();


    // Damp the ball when it collides with the dribbler. We damp each component
    // of the ball's momentum separately so we have the flexibility to tune this
    // behavior to match real life.
    Vector robot_facing_vector = Vector::createFromAngle(physics_robot->orientation());
    Vector robot_perp_vector   = robot_facing_vector.perpendicular();

    Vector ball_momentum          = physics_ball->momentum();
    Vector dribbler_perp_momentum = ball_momentum.project(robot_perp_vector);
    physics_ball->applyImpulse(-dribbler_perp_momentum * DRIBBLER_PERPENDICULAR_DAMPING);

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
                       PhysicsRobot::DRIBBLER_DEPTH);
    Vector dribble_force_vector = dribble_point - ball.position();
    Vector head_on_force        = dribble_force_vector.project(robot_facing_vector);
    Vector perp_force           = dribble_force_vector.project(robot_perp_vector);
    double head_on_dist_cm      = head_on_force.length() * 100;
    double perp_dist_cm         = perp_force.length() * 100;

    double perp_force_magnitude = 0.5 * std::pow(perp_dist_cm, 2);
    physics_ball->applyForce(perp_force.normalize(perp_force_magnitude));

    double head_on_magnitude = sigmoid(head_on_dist_cm, 0.15, 0.2) * dribbler_rpm /
                               physics_robot->robotConstants().max_force_dribbler_speed /
                               20;
    physics_ball->applyForce(head_on_force.normalize(head_on_magnitude));

    // Counteract the force pushing the ball into the robot so there is approximately 0
    // net force, so that the robot won't move due to dribbling
    physics_robot->applyForceToCenterOfMass(
        -(head_on_force.normalize(head_on_magnitude)));
}
