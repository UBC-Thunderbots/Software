#include "linear_velocity_agent.h"

LinearVelocityAgent::LinearVelocityAgent(Simulator *simulator, const Vector2 &position, float radius,
                                         const Vector2 &velocity, float maxSpeed,
                                         float maxAccel, std::size_t goalNo,
                                         float goalRadius)
    : Agent(simulator, position, radius, velocity, velocity, maxSpeed, maxAccel, goalNo, goalRadius)
{
}

void LinearVelocityAgent::computeNewVelocity()
{
    // New Velocity towards goal...?
    float new_x_vel = velocity_.getX() + maxAccel_ * simulator_->getTimeStep();
    float new_y_vel = velocity_.getY() + maxAccel_ * simulator_->getTimeStep();
    newVelocity_    = Vector2(new_x_vel, new_y_vel);
}
