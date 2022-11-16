#include "software/simulated_tests/non_terminating_validation_functions/robots_violating_motion_constraint.h"

#include "software/logger/logger.h"

void robotsViolatingMotionConstraint(
    std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield,
    std::shared_ptr<RobotNavigationObstacleFactory> obstacle_factory,
    TbotsProto::MotionConstraint constraint)
{
    std::vector<ObstaclePtr> obstacles =
        obstacle_factory->createStaticObstaclesFromMotionConstraint(constraint,
                                                                    world_ptr->field());
    for (const auto& robot : world_ptr->friendlyTeam().getAllRobots())
    {
        for (auto obstacle_ptr : obstacles)
        {
            if (obstacle_ptr.get()->contains(robot.position()))
            {
                yield("Robot " + std::to_string(robot.id()) +
                      " violated the motion constraint");
            }
        }
    }
}
