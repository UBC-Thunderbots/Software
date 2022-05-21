#include "linear_velocity_agent.h"

LinearVelocityAgent::LinearVelocityAgent(HRVOSimulator *simulator, const Vector &position,
                                         float radius, const Vector &velocity,
                                         float maxSpeed, float maxAccel, AgentPath &path)
    : Agent(simulator, position, radius, velocity, velocity, maxSpeed, maxAccel, path)
{
}

void LinearVelocityAgent::computeNewVelocity()
{
    // TODO (#2496): Fix bug where LinearVelocityAgents go past their destination
    // Preferring a velocity which points directly towards goal

    auto path_point_opt = path.getCurrentPathPoint();

    if (path_point_opt == std::nullopt)
    {
        pref_velocity_ = Vector(0.f, 0.f);
        new_velocity_  = Vector(0.f, 0.f);
        return;
    }

    Vector goal_pos = path_point_opt.value().getPosition();
    pref_velocity_  = goal_pos - position_;

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

VelocityObstacle LinearVelocityAgent::createVelocityObstacle(const Agent &other_agent)
{
    if ((position_ - other_agent.getPosition()).lengthSquared() >
        std::pow(radius_ + other_agent.getRadius(), 2))
    {
        // This Agent is not colliding with other agent
        Vector apex = velocity_;

        const float angle =
            (position_ - other_agent.getPosition()).orientation().toRadians();

        // opening angle = arcsin((rad_A + rad_B) / distance)
        const float openingAngle =
            std::asin((other_agent.getRadius() + radius_) /
                      (position_ - other_agent.getPosition()).length());

        // Direction of the two edges of the velocity obstacle
        Vector right_side =
            Vector(std::cos(angle - openingAngle), std::sin(angle - openingAngle));
        Vector left_side =
            Vector(std::cos(angle + openingAngle), std::sin(angle + openingAngle));
        return VelocityObstacle(apex, right_side, left_side);
    }
    // This Agent is colliding with other agent
    // Creates Velocity Obstacle with the sides being 180 degrees
    // apart from each other
    Vector apex = velocity_;
    Vector right_side =
        (other_agent.getPosition() - position_).perpendicular().normalize();
    Vector left_side = -right_side;
    return VelocityObstacle(apex, right_side, left_side);
}
