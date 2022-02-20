#include "software/simulated_tests/tactic_stepper.h"

#include "proto/message_translation/tbots_protobuf.h"
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
                                                                       const Robot& robot)
{
    std::vector<std::unique_ptr<Intent>> intents;
    auto intent = tactic->get(robot, world);
    intent->setMotionConstraints(motion_constraints);
    intents.push_back(std::move(intent));

    auto primitive_set_msg = navigator->getAssignedPrimitives(world, intents);
    return primitive_set_msg;
}
