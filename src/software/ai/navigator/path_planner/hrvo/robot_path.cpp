#include "robot_path.h"

#include <optional>

#include "path_point.h"

RobotPath::RobotPath()
{
    path_radius = 0.0f;
}

RobotPath::RobotPath(const std::vector<PathPoint> &path_points, double goal_radius_)
{
    path        = path_points;
    path_radius = goal_radius_;
}

void RobotPath::incrementPathIndex()
{
    curr_path_index++;
}

std::optional<PathPoint> RobotPath::getCurrentPathPoint() const
{
    if (curr_path_index >= path.size())
    {
        if (!path.empty())
        {
            return path[path.size() - 1];
        }
        else
        {
            return std::nullopt;
        }
    }
    else
    {
        return path[curr_path_index];
    }
}

bool RobotPath::isGoingToFinalPathPoint()
{
    return curr_path_index >= path.size() - 1;
}

double RobotPath::getPathRadius() const
{
    return path_radius;
}
