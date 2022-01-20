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

Agent::VelocityObstacle LinearVelocityAgent::createVelocityObstacle(const Agent &other_agent)
{
    VelocityObstacle velocityObstacle;
    if (absSq(position_ - other_agent.position_) > std::pow(radius_ + other_agent.radius_, 2))
    {
        // This Agent is not colliding with other agent
        velocityObstacle.apex_ = velocity_;

        const float angle = atan(position_ - other_agent.position_);

        // opening angle = arcsin((rad_A + rad_B) / distance)
        const float openingAngle =
                std::asin((other_agent.radius_ + radius_) / abs(position_ - other_agent.position_));

        // Direction of the two edges of the velocity obstacle
        velocityObstacle.side1_ =
                Vector2(std::cos(angle - openingAngle), std::sin(angle - openingAngle));
        velocityObstacle.side2_ =
                Vector2(std::cos(angle + openingAngle), std::sin(angle + openingAngle));
    }
    else
    {
        // This Agent is colliding with other agent
        // Creates Velocity Obstacle with the sides being 180 degrees
        // apart from each other
        velocityObstacle.apex_ = velocity_;
        velocityObstacle.side1_ = normal(other_agent.position_, position_);
        velocityObstacle.side2_ = -velocityObstacle.side1_;
    }
    return velocityObstacle;
}
