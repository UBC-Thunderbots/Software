#include "software/backend/simulation/simulator_ball.h"

SimulatorBall::SimulatorBall(std::weak_ptr<PhysicsBall> physics_ball) :physics_ball(physics_ball){
}

float SimulatorBall::getPositionX() const {
    if (auto ball = physics_ball.lock())
    {
        return ball->getBallWithTimestamp(Timestamp::fromSeconds(0)).position().x();
    }
    return 0.0;
}

float SimulatorBall::getPositionY() const {
    if (auto ball = physics_ball.lock())
    {
        return ball->getBallWithTimestamp(Timestamp::fromSeconds(0)).position().y();
    }
    return 0.0;
}

Point SimulatorBall::position() const {
    return Point(getPositionX(), getPositionY());
}

float SimulatorBall::getVelocityX() const {
    if (auto ball = physics_ball.lock())
    {
        return ball->getBallWithTimestamp(Timestamp::fromSeconds(0)).velocity().x();
    }
    return 0.0;
}

float SimulatorBall::getVelocityY() const {
    if (auto ball = physics_ball.lock())
    {
        return ball->getBallWithTimestamp(Timestamp::fromSeconds(0)).velocity().y();
    }
    return 0.0;
}

Vector SimulatorBall::velocity() const {
    return Vector(getVelocityX(), getVelocityY());
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

