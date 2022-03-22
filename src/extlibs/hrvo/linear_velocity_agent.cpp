#include "linear_velocity_agent.h"

LinearVelocityAgent::LinearVelocityAgent(HRVOSimulator *simulator, const Vector &position,
                                         float radius, const Vector &velocity,
                                         float maxSpeed, float maxAccel,
                                         std::size_t goal_index, float goalRadius)
    : Agent(simulator, position, radius, velocity, velocity, maxSpeed, maxAccel,
            goal_index, goalRadius)
{
}

void LinearVelocityAgent::computeNewVelocity()
{
    // TODO (#2496): Fix bug where LinearVelocityAgents go past their destination
    // Preferring a velocity which points directly towards goal
    pref_velocity_ = simulator_->goals[goal_index_]->getCurrentGoalPosition() - position_;

    if (pref_velocity_.length() > max_speed_)
    {
        pref_velocity_ = (pref_velocity_).normalize(max_speed_);
    }

    const Vector dv = pref_velocity_ - velocity_;
    if (dv.length() <= max_accel_ * simulator_->getTimeStep())
    {
        new_velocity_ = pref_velocity_;
    }
    else
    {
        // Calculate the maximum velocity towards the preferred velocity, given the
        // acceleration constraint
        new_velocity_ = velocity_ + dv.normalize(max_accel_ * simulator_->getTimeStep());
    }
}

Agent::VelocityObstacle LinearVelocityAgent::createVelocityObstacle(
    const Agent &other_agent)
{
    VelocityObstacle velocityObstacle;
    if ((position_ - other_agent.getPosition()).lengthSquared() >
        std::pow(radius_ + other_agent.getRadius(), 2))
    {
        // This Agent is not colliding with other agent
        velocityObstacle.apex_ = velocity_;

        const float angle =
            (position_ - other_agent.getPosition()).orientation().toRadians();

        // opening angle = arcsin((rad_A + rad_B) / distance)
        const float openingAngle =
            std::asin((other_agent.getRadius() + radius_) /
                      (position_ - other_agent.getPosition()).length());

        // Direction of the two edges of the velocity obstacle
        velocityObstacle.side1_ =
            Vector(std::cos(angle - openingAngle), std::sin(angle - openingAngle));
        velocityObstacle.side2_ =
            Vector(std::cos(angle + openingAngle), std::sin(angle + openingAngle));
    }
    else
    {
        // This Agent is colliding with other agent
        // Creates Velocity Obstacle with the sides being 180 degrees
        // apart from each other
        velocityObstacle.apex_ = velocity_;
        velocityObstacle.side1_ =
            (other_agent.getPosition() - position_).perpendicular().normalize();
        velocityObstacle.side2_ = -velocityObstacle.side1_;
    }
    return velocityObstacle;
}
