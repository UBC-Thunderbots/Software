#include "software/backend/simulation/simulator_ball.h"

SimulatorBall::SimulatorBall(std::weak_ptr<PhysicsBall> physics_ball) :physics_ball(physics_ball){}

float SimulatorBall::getPositionX() {
    if (auto ball = physics_ball.lock())
    {
        return ball->getBallWithTimestamp(Timestamp::fromSeconds(0)).position().x();
    }
    return 0.0;
}

float SimulatorBall::getPositionY() {
    if (auto ball = physics_ball.lock())
    {
        return ball->getBallWithTimestamp(Timestamp::fromSeconds(0)).position().y();
    }
    return 0.0;
}

float SimulatorBall::getVelocityX() {
    if (auto ball = physics_ball.lock())
    {
        return ball->getBallWithTimestamp(Timestamp::fromSeconds(0)).velocity().x();
    }
    return 0.0;
}

float SimulatorBall::getVelocityY() {
    if (auto ball = physics_ball.lock())
    {
        return ball->getBallWithTimestamp(Timestamp::fromSeconds(0)).velocity().y();
    }
    return 0.0;
}

void SimulatorBall::applyForce(const Vector &force) {
    if (auto ball = physics_ball.lock())
    {
        ball->applyForce(force);
    }
}

void SimulatorBall::applyImpulse(const Vector &impulse) {
    if (auto ball = physics_ball.lock())
    {
        ball->applyImpulse(impulse);
    }
}
