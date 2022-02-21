#include "software/simulated_tests/tactic_stepper.h"

#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"

TacticStepper::TacticStepper(std::shared_ptr<Tactic> tactic_to_run,
                             const std::set<MotionConstraint>& motion_constraints,
                             std::shared_ptr<const ThunderbotsConfig> thunderbots_config)
    : tactic(tactic_to_run),
      motion_constraints(motion_constraints),
      navigator(std::make_shared<Navigator>(
          std::make_unique<VelocityObstaclePathManager>(
              std::make_unique<ThetaStarPathPlanner>(),
              RobotNavigationObstacleFactory(
                  thunderbots_config->getRobotNavigationObstacleConfig())),
          RobotNavigationObstacleFactory(
              thunderbots_config->getRobotNavigationObstacleConfig()),
          thunderbots_config->getNavigatorConfig()))
{
}

std::unique_ptr<TbotsProto::PrimitiveSet> TacticStepper::getPrimitives(const World& world,
                                                                       unsigned robot_id)
{
    std::vector<std::unique_ptr<Intent>> intents;
    auto robot = world.friendlyTeam().getRobotById(robot_id);
    if (robot.has_value())
    {
        auto intent = tactic->get(robot.value(), world);
        intent->setMotionConstraints(motion_constraints);
        intents.push_back(std::move(intent));
        LOG(WARNING) << "step: " << robot_id;
    }
    else
    {
        LOG(WARNING) << "BRUH X_X WHY IS THERE NO ROBOT WITH ID " << robot_id;
    }

    auto primitive_set_msg = navigator->getAssignedPrimitives(world, intents);
    return primitive_set_msg;
}
