#include "lv_agent.h"

LVAgent::LVAgent(RobotId robot_id, const RobotState &robot_state, const RobotPath &path,
                 double radius, double max_speed, double max_accel,
                 double max_radius_inflation)
    : Agent(robot_id, robot_state, path, radius, max_speed, max_accel,
            max_radius_inflation)
{
}

void LVAgent::computeNewVelocity(
    const std::map<unsigned int, std::shared_ptr<Agent>> &robots, Duration time_step)
{
    Vector pref_velocity = computePreferredVelocity(time_step);
    setPreferredVelocity(pref_velocity);

    const Vector dv = preferred_velocity - velocity;
    if (dv.length() <= max_accel * time_step.toSeconds())
    {
        new_velocity = preferred_velocity;
    }
    else
    {
        // Calculate the maximum velocity towards the preferred velocity, given the
        // acceleration constraint
        new_velocity = velocity + dv.normalize(max_accel * time_step.toSeconds());
    }
}

Vector LVAgent::computePreferredVelocity(Duration time_step)
{
    auto path_point_opt = path.getCurrentPathPoint();

    if (path_point_opt == std::nullopt)
    {
        return Vector(0.f, 0.f);
    }
    Point goal_pos                 = path_point_opt.value().getPosition();
    Vector unbounded_pref_velocity = goal_pos - position;

    if (unbounded_pref_velocity.length() > max_speed)
    {
        return unbounded_pref_velocity.normalize(max_speed);
    }
    return unbounded_pref_velocity;
}

VelocityObstacle LVAgent::createVelocityObstacle(const Agent &other_agent)
{
    return VelocityObstacle::generateVelocityObstacle(
        Circle(Point(position), radius),
        Circle(Point(other_agent.getPosition()), other_agent.radius), velocity);
}

void LVAgent::updatePrimitive(const TbotsProto::Primitive &new_primitive,
                              const World &world, Duration time_step)
{
    // this operation is not supported for LV agents since they
    // represent enemy robots, which aren't controlled or updated by us
    return;
}
