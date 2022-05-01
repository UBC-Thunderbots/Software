#include "software/ai/navigator/path_planner/global_path_planner_factory.h"

GlobalPathPlannerFactory::GlobalPathPlannerFactory(
    const std::shared_ptr<const RobotNavigationObstacleConfig> navigation_obstacle_config,
    const Field &field)
{
    RobotNavigationObstacleFactory obstacle_factory =
        RobotNavigationObstacleFactory(navigation_obstacle_config);

    auto motion_constraint_enum_descriptor = TbotsProto::MotionConstraint_descriptor();

    // The idea of this is similar to Gray codes
    // (https://en.wikipedia.org/wiki/Gray_code#History_and_practical_application). The
    // idea is that each bit in the counter represents a MotionConstraint and whether it
    // is on and off. By cycling through every combination of bits, we'll consequently
    // also have every combination of MotionConstraints
    // (https://www.geeksforgeeks.org/generate-n-bit-gray-codes/)
    for (unsigned counter = 0;
         counter < std::pow(2, motion_constraint_enum_descriptor->value_count());
         ++counter)
    {
        std::set<TbotsProto::MotionConstraint> motion_constraint_obstacles;

        // Use the value of the counter and bit arithmetic to get the motion constraint
        // obstacles out
        unsigned constraint_bits     = counter;
        bool has_dynamic_constraints = false;
        for (int j = 0; j < motion_constraint_enum_descriptor->value_count();
             ++j, constraint_bits >>= 1)
        {
            if (constraint_bits & 1)
            {
                TbotsProto::MotionConstraint constraint;
                auto enum_value = motion_constraint_enum_descriptor->value(j);
                bool parsed =
                    TbotsProto::MotionConstraint_Parse(enum_value->name(), &constraint);
                CHECK(parsed) << "Couldn't parse MotionConstraint with value: "
                              << enum_value->name() << std::endl;

                if (enum_value->options().HasExtension(TbotsProto::dynamic) &&
                    enum_value->options().GetExtension(TbotsProto::dynamic))
                {
                    dynamic_motion_constraints.insert(constraint);
                    has_dynamic_constraints = true;
                    break;
                }
                motion_constraint_obstacles.emplace(constraint);
            }
        }

        if (!has_dynamic_constraints)
        {
            auto obstacles = obstacle_factory.createFromMotionConstraints(
                motion_constraint_obstacles, field);
            planners.emplace(std::make_pair(
                motion_constraint_obstacles,
                std::make_shared<EnlsvgPathPlanner>(field.fieldBoundary(), obstacles,
                                                    ROBOT_MAX_RADIUS_METERS)));
        }
    }
}

std::shared_ptr<const EnlsvgPathPlanner> GlobalPathPlannerFactory::getPathPlanner(
    std::set<TbotsProto::MotionConstraint> constraints) const
{
    for (const auto &dynamic_constraint : dynamic_motion_constraints)
    {
        constraints.erase(dynamic_constraint);
    }
    try
    {
        return planners.at(constraints);
    }
    catch (std::out_of_range &e)
    {
        LOG(WARNING)
            << "GlobalPathPlannerFactory is unable to obtain a path planner for the following motion constraints: ";
        for (auto constraint : constraints)
        {
            LOG(WARNING) << toString(constraint);
        }
        LOG(WARNING) << "Returning obstacle-free planner.";
    }
    return planners.at({});
}
