#include "global_path_planner.h"

GlobalPathPlanner::GlobalPathPlanner(const std::shared_ptr<const RobotNavigationObstacleConfig> navigation_obstacle_config, 
                                     const World &world, 
                                     const Rectangle &navigable_area) 
{
    RobotNavigationObstacleFactory obstacle_factory = RobotNavigationObstacleFactory(navigation_obstacle_config);
    std::vector<MotionConstraint> all_constraints = allValuesMotionConstraint();
    
    for (int counter = 0; counter < std::pow(2, all_constraints.size()); ++counter)
    {
        std::set<MotionConstraint> motion_constraint_obstacles;
        int constraint_bits = counter;
        for (unsigned j = 0; j < all_constraints.size(); ++j, constraint_bits = constraint_bits >> 1)
        {
            if (constraint_bits & 1)
            {
                motion_constraint_obstacles.emplace(all_constraints[j]);
            }
        }
        auto obstacles = obstacle_factory.createFromMotionConstraints(motion_constraint_obstacles, world);
        planners.emplace(std::make_pair(motion_constraint_obstacles,
                                        std::make_shared<EnlsvgPathPlanner>(navigable_area, 
                                                                            obstacles, 
                                                                            world.field().boundaryMargin())));
    }
}

std::shared_ptr<EnlsvgPathPlanner> GlobalPathPlanner::getPathGenerator(const World &world, const std::set<MotionConstraint> motion_constraints)
{
    return planners.at(motion_constraints);
}
