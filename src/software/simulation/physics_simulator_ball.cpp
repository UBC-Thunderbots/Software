#include "software/simulation/physics_simulator_ball.h"

#include "software/logger/logger.h"

PhysicsSimulatorBall::PhysicsSimulatorBall(std::weak_ptr<PhysicsBall> physics_ball)
    : physics_ball(physics_ball)
{
}

Point PhysicsSimulatorBall::checkValidAndReturnPoint(
    std::function<Point(const std::shared_ptr<PhysicsBall>)> func) const
{
    if (auto ball = physics_ball.lock())
    {
        return func(ball);
    }
    LOG(WARNING) << "PhysicsSimulatorBall being used with invalid PhysicsBall"
                 << std::endl;
    return Point(0, 0);
}

Vector PhysicsSimulatorBall::checkValidAndReturnVector(
    std::function<Vector(const std::shared_ptr<PhysicsBall>)> func) const
{
    if (auto ball = physics_ball.lock())
    {
        return func(ball);
    }
    LOG(WARNING) << "PhysicsSimulatorBall being used with invalid PhysicsBall"
                 << std::endl;
    return Vector(0, 0);
}

Point PhysicsSimulatorBall::position() const
{
    return checkValidAndReturnPoint([](auto ball) { return ball->position(); });
}

Vector PhysicsSimulatorBall::velocity() const
{
    return checkValidAndReturnVector([](auto ball) { return ball->velocity(); });
}
