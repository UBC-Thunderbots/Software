#include "software/simulated_tests/simulated_action_test_fixture.h"

#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"
#include "software/gui/drawing/navigator.h"
#include "software/proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "software/proto/message_translation/tbots_protobuf.h"

SimulatedActionTestFixture::SimulatedActionTestFixture()
    : navigator(std::make_shared<Navigator>(
          std::make_unique<VelocityObstaclePathManager>(
              std::make_unique<ThetaStarPathPlanner>(),
              RobotNavigationObstacleFactory(
                  DynamicParameters->getAIConfig()
                      ->getRobotNavigationObstacleFactoryConfig())),
          RobotNavigationObstacleFactory(DynamicParameters->getAIConfig()
                                             ->getRobotNavigationObstacleFactoryConfig()),
          DynamicParameters->getAIConfig()->getNavigatorConfig()))
{
}

void SimulatedActionTestFixture::SetUp()
{
    SimulatedTestFixture::SetUp();
    navigator = std::make_shared<Navigator>(
        std::make_unique<VelocityObstaclePathManager>(
            std::make_unique<ThetaStarPathPlanner>(),
            RobotNavigationObstacleFactory(
                DynamicParameters->getAIConfig()
                    ->getRobotNavigationObstacleFactoryConfig())),
        RobotNavigationObstacleFactory(
            DynamicParameters->getAIConfig()->getRobotNavigationObstacleFactoryConfig()),
        DynamicParameters->getAIConfig()->getNavigatorConfig());
}

void SimulatedActionTestFixture::setAction(std::shared_ptr<Action> action)
{
    if (action && action->getRobot())
    {
        this->action = action;
    }
    else
    {
        throw std::invalid_argument("Action does not contain a robot in the simulator");
    }
}


void SimulatedActionTestFixture::updatePrimitives(
    const World& world, std::shared_ptr<Simulator> simulator_to_update)
{
    std::vector<std::unique_ptr<Intent>> intents;
    if (auto new_robot = world.friendlyTeam().getRobotById(action->getRobot()->id()))
    {
        action->updateRobot(*world.friendlyTeam().getRobotById(action->getRobot()->id()));
    }
    else
    {
        LOG(FATAL) << "No robot with robot id " << action->getRobot()->id() << std::endl;
    }
    action->updateWorldParams(world);
    auto intent = action->getNextIntent();
    intent->setMotionConstraints(motion_constraints);
    intents.emplace_back(std::move(intent));

    auto primitive_set_msg = navigator->getAssignedPrimitives(world, intents);
    simulator_to_update->setYellowRobotPrimitiveSet(
        createNanoPbPrimitiveSet(*primitive_set_msg));
}


std::optional<PlayInfo> SimulatedActionTestFixture::getPlayInfo()
{
    return std::nullopt;
}

AIDrawFunction SimulatedActionTestFixture::getDrawFunctions()
{
    return drawNavigator(navigator);
}
