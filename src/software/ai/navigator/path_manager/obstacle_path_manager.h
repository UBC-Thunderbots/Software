#pragma once
#include "software/ai/navigator/path_manager/path_manager.h"

/**
 * ObstaclePathManager uses obstacles to arbitrate between paths to
 * implement the PathManager interface.
 */

class ObstaclePathManager : public PathManager
{
   public:
    std::vector<Path> getManagedPaths(std::vector<PathObjective> objectives,
                                      const Rectangle &navigable_area,
                                      const std::vector<Obstacle> &obstacles) override;

    explicit ObstaclePathManager(std::unique_ptr<PathPlanner> path_planner);

    // Path planner used to get paths
    std::unique_ptr<PathPlanner> path_planner;
};
