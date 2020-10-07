#pragma once

#include <vector>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/world/robot.h"

/**
 * Path objective is used to pass information from navigator
 * to path manager to plan paths
 */
class PathObjective
{
   public:
    PathObjective() = delete;
    PathObjective(const Point start, const Point end, const double current_speed,
                  const std::vector<ObstaclePtr> &obstacles, RobotId robot_id)
        : robot_id(robot_id),
          start(start),
          end(end),
          current_speed(current_speed),
          obstacles(obstacles)
    {
    }

    PathObjective(const PathObjective &other)
        : robot_id(other.robot_id),
          start(other.start),
          end(other.end),
          current_speed(other.current_speed),
          obstacles(other.obstacles)
    {
    }

    const RobotId robot_id;
    const Point start;
    const Point end;
    const double current_speed;
    const std::vector<ObstaclePtr> obstacles;

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
