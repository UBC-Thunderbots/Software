#include "software/simulation/physics_simulator_ball.h"

#include "software/logger/logger.h"

PhysicsSimulatorBall::PhysicsSimulatorBall(std::weak_ptr<PhysicsBall> physics_ball)
    : physics_ball(physics_ball)
{
}

Point PhysicsSimulatorBall::position() const
{
    return checkValidAndExecute<Point>([](auto ball) { return ball->position(); });
}

Vector PhysicsSimulatorBall::velocity() const
{
    return checkValidAndExecute<Vector>([](auto ball) { return ball->velocity(); });
}
