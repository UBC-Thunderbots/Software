#include "software/ai/ai.h"

#include <chrono>

#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/stp.h"
#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"

AI::AI(std::shared_ptr<const AiConfig> ai_config,
       std::shared_ptr<const AiControlConfig> control_config)
    : navigator(std::make_shared<Navigator>(
          std::make_unique<VelocityObstaclePathManager>(
              std::make_unique<ThetaStarPathPlanner>(),
              RobotNavigationObstacleFactory(
                  ai_config->getRobotNavigationObstacleFactoryConfig())),
          RobotNavigationObstacleFactory(
              ai_config->getRobotNavigationObstacleFactoryConfig()),
          ai_config->getNavigatorConfig())),
      // We use the current time in nanoseconds to initialize STP with a "random" seed
      high_level(std::make_unique<STP>(
          []() { return std::make_unique<HaltPlay>(); }, control_config,
          std::chrono::system_clock::now().time_since_epoch().count()))
{
}

std::unique_ptr<TbotsProto::PrimitiveSet> AI::getPrimitives(const World &world) const
{
    std::vector<std::unique_ptr<Intent>> assigned_intents = high_level->getIntents(world);

    return navigator->getAssignedPrimitives(world, assigned_intents);
}

PlayInfo AI::getPlayInfo() const
{
    return high_level->getPlayInfo();
}

std::shared_ptr<Navigator> AI::getNavigator() const
{
    return navigator;
}
