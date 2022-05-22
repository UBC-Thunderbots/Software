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
	return VelocityObstacle::generateVelocityObstacle(Circle(Point(getPosition()), getRadius()), Circle(Point(other_agent.getPosition()), other_agent.getRadius()), getVelocity());
}

