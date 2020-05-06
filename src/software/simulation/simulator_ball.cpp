#include "software/simulation/simulator_ball.h"

#include "software/logger/logger.h"

SimulatorBall::SimulatorBall(std::weak_ptr<PhysicsBall> physics_ball)
    : physics_ball(physics_ball)
{
}

Point SimulatorBall::checkValidAndReturnPoint(
    std::function<Point(const std::shared_ptr<PhysicsBall>)> func) const
{
    if (auto ball = physics_ball.lock())
    {
        return func(ball);
    }
    LOG(WARNING) << "SimulatorBall being used with invalid PhysicsBall" << std::endl;
    return Point(0, 0);
}

Vector SimulatorBall::checkValidAndReturnVector(
    std::function<Vector(const std::shared_ptr<PhysicsBall>)> func) const
{
    if (auto ball = physics_ball.lock())
    {
        return func(ball);
    }
    LOG(WARNING) << "SimulatorBall being used with invalid PhysicsBall" << std::endl;
    return Vector(0, 0);
}

Point SimulatorBall::position() const
{
    return checkValidAndReturnPoint([](auto ball) { return ball->position(); });
}

Vector SimulatorBall::velocity() const
{
    return checkValidAndReturnVector([](auto ball) { return ball->velocity(); });
}
