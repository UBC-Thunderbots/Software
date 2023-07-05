#include "software/ai/navigator/path_planner/global_path_planner_factory.h"

GlobalPathPlannerFactory::GlobalPathPlannerFactory(
    const TbotsProto::RobotNavigationObstacleConfig navigation_obstacle_config,
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
        std::set<TbotsProto::MotionConstraint> static_motion_constraints;

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
                static_motion_constraints.emplace(constraint);
            }
        }

        if (!has_dynamic_constraints)
        {
            auto obstacles = obstacle_factory.createStaticObstaclesFromMotionConstraints(
                static_motion_constraints, field);
            planners.emplace(std::make_pair(static_motion_constraints,
                                            std::make_shared<EnlsvgPathPlanner>(
                                                field.fieldBoundary(), obstacles, 0)));

            google::protobuf::RepeatedPtrField<TbotsProto::Obstacles>
                repeated_obstacle_proto;
            for (const auto &obstacle : obstacles)
            {
                *(repeated_obstacle_proto.Add()) = obstacle->createObstacleProto();
            }

            motion_constraint_to_obstacles.emplace(
                std::make_pair(static_motion_constraints, repeated_obstacle_proto));
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

google::protobuf::RepeatedPtrField<TbotsProto::Obstacles>
GlobalPathPlannerFactory::getStaticObstacles(
    std::set<TbotsProto::MotionConstraint> constraints) const
{
    auto obstacles = motion_constraint_to_obstacles.at({});
    for (const auto &dynamic_constraint : dynamic_motion_constraints)
    {
        constraints.erase(dynamic_constraint);
    }
    try
    {
        obstacles = motion_constraint_to_obstacles.at(constraints);
    }
    catch (std::out_of_range &e)
    {
        LOG(WARNING)
            << "GlobalPathPlannerFactory is unable to obtain obstacles for the following motion constraints: ";
        for (auto constraint : constraints)
        {
            LOG(WARNING) << toString(constraint);
        }
        LOG(WARNING) << "Returning no obstacles.";
    }
    return obstacles;
}
