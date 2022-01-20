#include "linear_velocity_agent.h"

LinearVelocityAgent::LinearVelocityAgent(Simulator *simulator, const Vector2 &position,
                                         float radius, const Vector2 &velocity,
                                         float maxSpeed, float maxAccel,
                                         std::size_t goalNo, float goalRadius)
    : Agent(simulator, position, radius, velocity, velocity, maxSpeed, maxAccel, goalNo,
            goalRadius)
{
}

void LinearVelocityAgent::computeNewVelocity()
{
    // New Velocity towards goal...?
    // TODO: Limit velocity to maxSpeed
    Vector2 velocity_to_dest =
        simulator_->goals_[goalNo_]->getCurrentGoalPosition() - position_;
    const float dv = abs(velocity_to_dest - velocity_);
    if (dv != 0.f)
    {
        newVelocity_ = (1.0f - (maxAccel_ * simulator_->timeStep_ / dv)) * velocity_ +
                         (maxAccel_ * simulator_->timeStep_ / dv) * velocity_to_dest;
    }
    else
    {
        newVelocity_ = velocity_;
    }
}
