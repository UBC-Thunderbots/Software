#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"

VelocityObstaclePathManager::VelocityObstaclePathManager(
    std::unique_ptr<PathPlanner> path_planner)
    : path_planner(std::move(path_planner))
{
}

const std::map<RobotId, std::optional<Path>> VelocityObstaclePathManager::getManagedPaths(
    const std::unordered_set<PathObjective> &objectives, const Rectangle &navigable_area)
{
    std::map<RobotId, std::optional<Path>> managed_paths;

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
            objectives, current_objective, robot_obstacle_inflation);
        path_obstacles.insert(path_obstacles.end(), current_velocity_obstacles.begin(),
                              current_velocity_obstacles.end());
        path_obstacles.insert(path_obstacles.end(), current_objective.obstacles.begin(),
                              current_objective.obstacles.end());
        auto path = path_planner->findPath(current_objective.start, current_objective.end,
                                           navigable_area, path_obstacles);

        // store path in managed_paths
        managed_paths.insert({current_objective.robot_id, path});

        // store velocity obstacle for current path
        if (path && path->size() >= 2)
        {
            std::vector<Point> path_points = path->getKnots();
            current_velocity_obstacles.emplace_back(
                Obstacle::createVelocityObstacleWithScalingParams(
                    current_objective.start, path_points[1],
                    current_objective.current_speed, robot_obstacle_inflation,
                    velocity_obstacle_inflation));
        }
    }

    return managed_paths;
}

const std::vector<Obstacle>
VelocityObstaclePathManager::getObstaclesAroundStartOfOtherObjectives(
    const std::unordered_set<PathObjective> &objectives,
    const PathObjective &current_objective, double inflation_factor)
{
    std::vector<Obstacle> obstacles;
    for (auto const &obj : objectives)
    {
        if (obj != current_objective)
        {
            obstacles.push_back(Obstacle::createCircleObstacle(
                obj.start, ROBOT_MAX_RADIUS_METERS, inflation_factor));
        }
    }
    return obstacles;
}
