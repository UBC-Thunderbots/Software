#pragma once
#include <set>
#include <vector>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/path_planner/path_planner.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/geom/spline.h"

/**
 * PathManager is an interface for a path manager that,
 * given a list of start and destination point pairs,
 * a list of static obstacles, and navigable_area will return
 * a list of 'best' paths between each of the pairs of
 * start and destination
 */

// Used to plan a path for a robot
class PathObjective
{
   public:
    PathObjective(const Point start, const Point end, const double current_velocity,
                  const std::vector<Obstacle> &obstacles, RobotId robot_id)
        : start(start),
          end(end),
          current_velocity(current_velocity),
          obstacles(obstacles),
          robot_id(robot_id)
    {
    }

    PathObjective(const PathObjective &other)
        : start(other.start),
          end(other.end),
          current_velocity(other.current_velocity),
          obstacles(other.obstacles),
          robot_id(other.robot_id)
    {
    }

    const RobotId robot_id;
    const Point start;
    const Point end;
    const double current_velocity;
    const std::vector<Obstacle> obstacles;

    bool operator==(const PathObjective &other) const
    {
        return robot_id == other.robot_id;
    }

    bool operator!=(const PathObjective &other) const
    {
        return robot_id != other.robot_id;
    }

    bool operator>(const PathObjective &other) const
    {
        return robot_id > other.robot_id;
    }

    bool operator<(const PathObjective &other) const
    {
        return robot_id < other.robot_id;
    }
};

class PathManager
{
   public:
    /**
     * Returns map of robot ids to paths guided by path objectives,
     * given navigable_area and static_obstacles
     *
     * @param objectives map of robot id to path objectives
     * @param navigable_area Rectangle representing the navigable area
     * @param static_obstacles static obstacles to avoid, excluding the robots
     *  whose paths are being planned
     *
     * @return map of robot ids to paths
     */
    virtual const std::map<PathObjective, Path> getManagedPaths(
        const std::set<PathObjective> &objectives, const Rectangle &navigable_area,
        const std::vector<Obstacle> &static_obstacles) = 0;

    virtual ~PathManager() = default;
};
