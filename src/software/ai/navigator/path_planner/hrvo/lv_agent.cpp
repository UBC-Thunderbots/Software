#include "lv_agent.h"

LVAgent::LVAgent(RobotId robot_id, RobotState robot_state, TeamSide side, RobotPath &path,
                 double radius, double max_speed, double max_accel,
                 double max_radius_inflation)
    : Agent(robot_id, robot_state, side, path, radius, max_speed, max_accel,
            max_radius_inflation)
{
}

void LVAgent::computeNewVelocity(std::map<unsigned int, std::shared_ptr<Agent>> &robots,
                                 double time_step)
{
    // TODO (#2496): Fix bug where LinearVelocityAgents go past their destination
    // Preferring a velocity which points directly towards goal

    Vector pref_velocity = computePreferredVelocity(time_step);
    setPreferredVelocity(pref_velocity);

    const Vector dv = preferred_velocity - robot_state.velocity();
    if (dv.length() <= max_accel * time_step)
    {
        new_velocity = preferred_velocity;
    }
    else
    {
        // Calculate the maximum velocity towards the preferred velocity, given the
        // acceleration constraint
        new_velocity = robot_state.velocity() + dv.normalize(max_accel * time_step);
    }
}

Vector LVAgent::computePreferredVelocity(double time_step)
{
    auto path_point_opt = path.getCurrentPathPoint();

    if (path_point_opt == std::nullopt)
    {
        return Vector(0.f, 0.f);
    }
    Point goal_pos                 = path_point_opt.value().getPosition();
    Vector unbounded_pref_velocity = goal_pos - robot_state.position();

    if (unbounded_pref_velocity.length() > max_speed)
    {
        return unbounded_pref_velocity.normalize(max_speed);
    }
    return unbounded_pref_velocity;
}

VelocityObstacle LVAgent::createVelocityObstacle(const Agent &other_agent)
{
    return VelocityObstacle::generateVelocityObstacle(
        Circle(Point(robot_state.position()), radius),
        Circle(Point(other_agent.robot_state.position()), other_agent.radius),
        robot_state.velocity());
}

void LVAgent::updatePrimitive(const TbotsProto::Primitive &new_primitive,
                              const World &world, double time_step)
{
    return;
}
