#pragma once

#include <vector>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/world/robot.h"

/**
 * Path objective is used to pass information from navigator
 * to path manager to plan paths
 */
struct PathObjective
{
    const RobotId robot_id;
    const Point start;
    const Point end;
    const double current_speed;
    std::vector<ObstaclePtr> obstacles;

    bool operator==(const PathObjective &other) const
    {
        return robot_id == other.robot_id;
    }

    bool operator!=(const PathObjective &other) const
    {
        return robot_id != other.robot_id;
    }
};

template <>
struct std::hash<PathObjective> final
{
    std::size_t operator()(const PathObjective &path_objective) const
    {
        std::hash<RobotId> h;
        return h(path_objective.robot_id);
    }
};
