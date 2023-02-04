#include "lv_agent.h"

LVAgent::LVAgent(RobotId robot_id, const RobotState &robot_state, TeamSide side, RobotPath &path,
                 double radius, double min_radius, double max_speed, double max_accel, double max_radius_inflation) :
        robot_id(robot_id), robot_state(robot_state), side(side), path(path), radius(radius),
        Agent(min_radius, max_speed, max_accel, max_radius_inflation)
{
}

void LVAgent::updateRadiusFromVelocity() {
    // Linearly increase radius based on the current agent velocity
    radius = min_radius + max_radius_inflation * (robot_state.velocity().length() / max_speed);
}

const RobotPath &LVAgent::getPath()
{
    return path;
}

