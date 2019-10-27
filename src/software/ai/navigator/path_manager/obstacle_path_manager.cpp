#include "software/ai/navigator/path_manager/obstacle_path_manager.h"

/**
 * This file contains the implementation of the obstacle path manager
 * which uses obstacles to arbitrate between paths
 * ret a path consisting of only the start and destination
 * points.
 */


ObstaclePathManager::ObstaclePathManager(std::unique_ptr<PathPlanner> path_planner)
    : path_planner(std::move(path_planner))
{
}

std::vector<Path> ObstaclePathManager::getManagedPaths(
    std::vector<PathObjective> objectives, const Rectangle &navigable_area,
    const std::vector<Obstacle> &obstacles)
{

    std::vector<Obstacle> current_velocity_obstacles;

    for (auto obj : objectives)
    {
        Path path =
            path_planner->findPath(obj.start, obj.end, navigable_area, obstacles);
    }

    return std::vector<Path>({Path(std::nullopt)});
}
