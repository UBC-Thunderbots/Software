#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"

VelocityObstaclePathManager::VelocityObstaclePathManager(
    std::unique_ptr<PathPlanner> path_planner,
    RobotNavigationObstacleFactory robot_navigation_obstacle_factory)
    : path_planner(std::move(path_planner)),
      robot_navigation_obstacle_factory(std::move(robot_navigation_obstacle_factory))
{
}

const std::map<RobotId, std::optional<Path>> VelocityObstaclePathManager::getManagedPaths(
    const std::unordered_set<PathObjective> &objectives, const Rectangle &navigable_area)
{
    std::map<RobotId, std::optional<Path>> managed_paths;
    path_planning_obstacles.clear();

    // Velocity obstacles used to avoid collisions.
    // As we plan a path for each robot, a corresponding obstacle will be added
    // to this list so that paths planned later do not collide with the path we just
    // planned. Please see: https://en.wikipedia.org/wiki/Velocity_obstacle
    std::vector<ObstaclePtr> current_velocity_obstacles;

    for (auto const &current_objective : objectives)
    {
        // find path with relevant obstacles
        std::vector<ObstaclePtr> path_obstacles = current_objective.obstacles;
        path_planning_obstacles.insert(path_planning_obstacles.end(),
                                       path_obstacles.begin(), path_obstacles.end());
        auto path = path_planner->findPath(current_objective.start, current_objective.end,
                                           navigable_area, path_obstacles);

        // store path in managed_paths
        managed_paths.insert({current_objective.robot_id, path});

        // store velocity obstacle for current path
        if (path && path->getNumKnots() >= 2)
        {
            // We want to avoid the start of every other path, assuming that
            // there is a robot moving along the path from the path's start
            std::vector<Point> path_points = path->getKnots();
            Vector initial_path_velocity =
                (path_points[1] - current_objective.start)
                    .normalize(current_objective.current_speed);
            Robot mock_path_robot(0, current_objective.start, initial_path_velocity,
                                  Angle::zero(), AngularVelocity::zero(),
                                  Timestamp::fromSeconds(0));
            current_velocity_obstacles.emplace_back(
                robot_navigation_obstacle_factory.createFromRobot(mock_path_robot));
        }
    }

    return managed_paths;
}

const std::vector<ObstaclePtr> VelocityObstaclePathManager::getObstacles(void) const
{
    return path_planning_obstacles;
}

const std::vector<ObstaclePtr>
VelocityObstaclePathManager::getObstaclesAroundStartOfOtherObjectives(
    const std::unordered_set<PathObjective> &objectives,
    const PathObjective &current_objective)
{
    std::vector<ObstaclePtr> obstacles;
    for (auto const &obj : objectives)
    {
        if (obj != current_objective)
        {
            obstacles.push_back(
                robot_navigation_obstacle_factory.createFromRobotPosition(obj.start));
        }
    }
    return obstacles;
}
