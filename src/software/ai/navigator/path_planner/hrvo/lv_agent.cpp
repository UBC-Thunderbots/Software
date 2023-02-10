#include "lv_agent.h"

LVAgent::LVAgent(RobotId robot_id, const RobotState &robot_state, TeamSide side, RobotPath &path,
                 double radius, double min_radius, double max_speed, double max_accel, double max_radius_inflation) :
        Agent(robot_id, robot_state, side,
              path, radius, min_radius,
              max_speed, max_accel, max_radius_inflation)
{
}
