#include "linear_velocity_agent.h"

LinearVelocityAgent::LinearVelocityAgent(Simulator *simulator, const Vector2 &position,
                                         float radius, const Vector2 &velocity,
                                         float maxSpeed, float maxAccel,
                                         std::size_t goal_index, float goalRadius)
    : Agent(simulator, position, radius, velocity, velocity, maxSpeed, maxAccel, goal_index,
            goalRadius)
{
}

void LinearVelocityAgent::computeNewVelocity()
{
    // New Velocity towards goal...?
    // TODO: Limit velocity to maxSpeed
    Vector2 velocity_to_dest =
            simulator_->goals_[goal_index_]->getCurrentGoalPosition() - position_;
    const float dv = abs(velocity_to_dest - velocity_);
    if (dv != 0.f)
    {
        new_velocity_ = normalize(velocity_) * max_speed_;
//                (1.0f - (max_accel_ * simulator_->timeStep_ / dv)) * velocity_ +
//                         (max_accel_ * simulator_->timeStep_ / dv) * velocity_to_dest;
    }
    else
    {
        new_velocity_ = velocity_;
    }
}

Agent::VelocityObstacle LinearVelocityAgent::createVelocityObstacle(const Agent &other_agent)
{
    VelocityObstacle velocityObstacle;
    if (absSq(position_ - other_agent.getPosition()) > std::pow(radius_ + other_agent.getRadius(), 2))
    {
        // This Agent is not colliding with other agent
        velocityObstacle.apex_ = velocity_;

        const float angle = atan(position_ - other_agent.getPosition());

        // opening angle = arcsin((rad_A + rad_B) / distance)
        const float openingAngle =
                std::asin((other_agent.getRadius() + radius_) / abs(position_ - other_agent.getPosition()));

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
        velocityObstacle.side1_ = normal(other_agent.getPosition(), position_);
        velocityObstacle.side2_ = -velocityObstacle.side1_;
    }
    return velocityObstacle;
}
