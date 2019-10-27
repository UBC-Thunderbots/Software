#pragma once
#include "software/ai/navigator/path_manager/path_manager.h"
#include "software/util/parameter/dynamic_parameters.h"

/**
 * ObstaclePathManager uses static_obstacles to arbitrate between paths to
 * implement the PathManager interface.
 */

class ObstaclePathManager : public PathManager
{
   public:
    std::vector<Path> getManagedPaths(
        std::vector<PathObjective> objectives, const Rectangle &navigable_area,
        const std::vector<Obstacle> &static_obstacles) override;

    /**
     * Creates obstacles around the start of objectives
     * except for current_index
     *
     * @param objectives objectives to make obstacles
     * @param current_index objective to skip
     * @param inflation_factor how much to inflate obstacle
     *
     * @return list of obstacles that around other objectives' starts
     */
    std::vector<Obstacle> getObstaclesFromOtherObjectives(
        std::vector<PathObjective> objectives, size_t current_index,
        double inflation_factor);

    explicit ObstaclePathManager(std::unique_ptr<PathPlanner> path_planner);

    // Path planner used to get paths
    std::unique_ptr<PathPlanner> path_planner;
};
