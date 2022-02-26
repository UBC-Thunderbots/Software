#include "software/ai/ai.h"

#include <chrono>

#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/stp.h"
#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"

AI::AI(std::shared_ptr<const AiConfig> ai_config)
    : navigator(std::make_shared<Navigator>(
          std::make_unique<VelocityObstaclePathManager>(
              std::make_unique<ThetaStarPathPlanner>(),
              RobotNavigationObstacleFactory(
                  ai_config->getRobotNavigationObstacleConfig())),
          RobotNavigationObstacleFactory(ai_config->getRobotNavigationObstacleConfig()),
          ai_config->getNavigatorConfig())),
      // We use the current time in nanoseconds to initialize STP with a "random" seed
      stp(std::make_unique<STP>(ai_config))
{
}

void AI::overridePlayConstructor(std::optional<PlayConstructor> constructor)
{
    stp->overridePlayConstructor(constructor);
}

std::unique_ptr<TbotsProto::PrimitiveSet> AI::getPrimitives(const World &world) const
{
    std::vector<std::unique_ptr<Intent>> assigned_intents = stp->getIntents(world);

    return navigator->getAssignedPrimitives(world, assigned_intents);
}

TbotsProto::PlayInfo AI::getPlayInfo() const
{
    return stp->getPlayInfo();
}

std::shared_ptr<Navigator> AI::getNavigator() const
{
    return navigator;
}
