#include "linear_velocity_agent.h"

LinearVelocityAgent::LinearVelocityAgent(Simulator *simulator, const Vector2 &position,
                                         float radius, const Vector2 &velocity,
                                         float maxSpeed, float maxAccel,
                                         std::size_t goal_index, float goalRadius)
    : Agent(simulator, position, radius, velocity, velocity, maxSpeed, maxAccel,
            goal_index, goalRadius)
{
}

void LinearVelocityAgent::computeNewVelocity()
{
    pref_velocity_ =
        simulator_->goals_[goal_index_]->getCurrentGoalPosition() - position_;

    if (abs(pref_velocity_) > max_speed_)
    {
        pref_velocity_ = normalize(pref_velocity_) * max_speed_;
    }

    const float dv = abs(pref_velocity_ - velocity_);
    if (dv <= max_accel_)
    {
        new_velocity_ = pref_velocity_;
    }
    else
    {
        // TODO: Might be able to simplify to:
        //       = velocity_ + max_accel_ * (unit vector dv)
        // Calculate the max velocity (given max accel) in the direction of dv
        // Should only do this if dv > max_accel_ to we don't go faster than accel

        // Calculate the maximum velocity towards the preferred velocity, given the
        // max acceleration constraint
        new_velocity_ = velocity_ + (max_accel_ * simulator_->timeStep_) * (dv / abs(dv)); // TODO: Compare the two outputs
        new_velocity_ = (1.0f - (max_accel_ * simulator_->timeStep_ / dv)) * velocity_ +
                        (max_accel_ * simulator_->timeStep_ / dv) * pref_velocity_;
    }
}

Agent::VelocityObstacle LinearVelocityAgent::createVelocityObstacle(
    const Agent &other_agent)
{
    VelocityObstacle velocityObstacle;
    if (absSq(position_ - other_agent.getPosition()) >
        std::pow(radius_ + other_agent.getRadius(), 2))
    {
        // This Agent is not colliding with other agent
        velocityObstacle.apex_ = velocity_;

        const float angle = atan(position_ - other_agent.getPosition());

        // opening angle = arcsin((rad_A + rad_B) / distance)
        const float openingAngle = std::asin((other_agent.getRadius() + radius_) /
                                             abs(position_ - other_agent.getPosition()));

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
        velocityObstacle.apex_  = velocity_;
        velocityObstacle.side1_ = normal(other_agent.getPosition(), position_);
        velocityObstacle.side2_ = -velocityObstacle.side1_;
    }
    return velocityObstacle;
}
