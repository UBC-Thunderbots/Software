#include "global_path_planner.h"

GlobalPathPlanner::GlobalPathPlanner(const std::shared_ptr<const RobotNavigationObstacleConfig> navigation_obstacle_config, 
                                     const World &world, 
                                     const Rectangle &navigable_area) 
{
    RobotNavigationObstacleFactory obstacle_factory = RobotNavigationObstacleFactory(navigation_obstacle_config);
    std::vector<MotionConstraint> all_constraints = allValuesMotionConstraint();
    
    // The idea of this is similar to Gray codes 
    // (https://en.wikipedia.org/wiki/Gray_code#History_and_practical_application). The idea is that each bit in the 
    // counter represents a MotionConstraint and whether it is on and off. By cycling through every combination of bits,
    // we'll consequently also have every combination of MotionConstraints
    // (https://www.geeksforgeeks.org/generate-n-bit-gray-codes/)
    for (int counter = 0; counter < std::pow(2, all_constraints.size()); ++counter)
    {
        std::set<MotionConstraint> motion_constraint_obstacles;
        
        // Use the value of the counter and bit arithmetic to get the motion constraint obstacles out
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

std::shared_ptr<EnlsvgPathPlanner> GlobalPathPlanner::getPathGenerator(
    const std::set<MotionConstraint> motion_constraints)
{
    try
    {
        return planners.at(motion_constraints);
    }
    catch (std::out_of_range &e)
    {
        LOG(WARNING) << "GlobalPathPlanner is unable to obtain a path planner for the following motion constraints: ";
        for (auto constraint : motion_constraints)
        {
            LOG(WARNING) << toString(constraint) << " ";
        }
        LOG(WARNING) << "\nReturning obstacle-free planner." << std::endl;
    }
    return planners.at({});
}
