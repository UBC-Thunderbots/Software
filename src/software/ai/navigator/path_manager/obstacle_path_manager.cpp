#include "software/ai/navigator/path_manager/obstacle_path_manager.h"

ObstaclePathManager::ObstaclePathManager(std::unique_ptr<PathPlanner> path_planner)
    : path_planner(std::move(path_planner))
{
}

const std::map<RobotId, Path> ObstaclePathManager::getManagedPaths(
    const std::map<RobotId, PathObjective> &objectives, const Rectangle &navigable_area,
    const std::vector<Obstacle> &static_obstacles)
{
    std::map<RobotId, Path> managed_paths;

    // Velocity obstacles used to avoid collisions.
    // As we plan a path for each robot, a corresponding obstacle will be added
    // to this list so that paths planned later do not collide with the path we just
    // planned. Please see: https://en.wikipedia.org/wiki/Velocity_obstacle
    std::vector<Obstacle> current_velocity_obstacles;

    // cache dynamic params
    double robot_obstacle_inflation = Util::DynamicParameters->getNavigatorConfig()
                                          ->RobotObstacleInflationFactor()
                                          ->value();
    double velocity_obstacle_inflation = Util::DynamicParameters->getNavigatorConfig()
                                             ->VelocityObstacleInflationFactor()
                                             ->value();

    for (auto const &current_objective : objectives)
    {
        // find path with relevant obstacles
        std::vector<Obstacle> path_obstacles = getObstaclesAroundStartOfOtherObjectives(
            objectives, current_objective.first, robot_obstacle_inflation);
        path_obstacles.insert(path_obstacles.end(), current_velocity_obstacles.begin(),
                              current_velocity_obstacles.end());
        path_obstacles.insert(path_obstacles.end(),
                              current_objective.second.obstacles.begin(),
                              current_objective.second.obstacles.end());
        Path path = path_planner->findPath(current_objective.second.start,
                                           current_objective.second.end, navigable_area,
                                           path_obstacles);

        // store path in managed_paths
        managed_paths.insert({current_objective.first, path});

        // store velocity obstacle for current path
        if (path && path->size() >= 2)
        {
            std::vector<Point> path_points = path->getKnots();
            current_velocity_obstacles.emplace_back(
                Obstacle::createVelocityObstacleWithScalingParams(
                    current_objective.second.start, path_points[1],
                    current_objective.second.current_velocity, robot_obstacle_inflation,
                    velocity_obstacle_inflation));
        }
    }

    return managed_paths;
}

const std::vector<Obstacle> ObstaclePathManager::getObstaclesAroundStartOfOtherObjectives(
    const std::map<RobotId, PathObjective> &objectives, size_t current_index,
    double inflation_factor)
{
    std::vector<Obstacle> obstacles;
    for (auto const &obj : objectives)
    {
        if (obj.first != current_index)
        {
            obstacles.push_back(Obstacle::createCircleObstacle(
                obj.second.start, ROBOT_MAX_RADIUS_METERS, inflation_factor));
        }
    }
    return obstacles;
}
