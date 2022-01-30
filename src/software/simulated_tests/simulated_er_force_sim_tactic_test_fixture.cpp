#include "software/simulated_tests/simulated_er_force_sim_tactic_test_fixture.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"
#include "software/gui/drawing/navigator.h"
#include "software/simulation/er_force_simulator.h"
#include "software/test_util/test_util.h"

SimulatedErForceSimTacticTestFixture::SimulatedErForceSimTacticTestFixture()
    : friendly_motion_constraints(),
      enemy_motion_constraints(),
      friendly_navigator(std::make_shared<Navigator>(
          std::make_unique<VelocityObstaclePathManager>(
              std::make_unique<ThetaStarPathPlanner>(),
              RobotNavigationObstacleFactory(
                  friendly_thunderbots_config->getRobotNavigationObstacleConfig())),
          RobotNavigationObstacleFactory(
              friendly_thunderbots_config->getRobotNavigationObstacleConfig()),
          friendly_thunderbots_config->getNavigatorConfig())),
      enemy_navigator(std::make_shared<Navigator>(
          std::make_unique<VelocityObstaclePathManager>(
              std::make_unique<ThetaStarPathPlanner>(),
              RobotNavigationObstacleFactory(
                  enemy_thunderbots_config->getRobotNavigationObstacleConfig())),
          RobotNavigationObstacleFactory(
              enemy_thunderbots_config->getRobotNavigationObstacleConfig()),
          enemy_thunderbots_config->getNavigatorConfig()))
{
}

void SimulatedErForceSimTacticTestFixture::SetUp()
{
    SimulatedErForceSimTestFixture::SetUp();
    friendly_navigator = std::make_shared<Navigator>(
        std::make_unique<VelocityObstaclePathManager>(
            std::make_unique<ThetaStarPathPlanner>(),
            RobotNavigationObstacleFactory(
                friendly_thunderbots_config->getRobotNavigationObstacleConfig())),
        RobotNavigationObstacleFactory(
            friendly_thunderbots_config->getRobotNavigationObstacleConfig()),
        friendly_thunderbots_config->getNavigatorConfig());
    enemy_navigator = std::make_shared<Navigator>(
        std::make_unique<VelocityObstaclePathManager>(
            std::make_unique<ThetaStarPathPlanner>(),
            RobotNavigationObstacleFactory(
                enemy_thunderbots_config->getRobotNavigationObstacleConfig())),
        RobotNavigationObstacleFactory(
            enemy_thunderbots_config->getRobotNavigationObstacleConfig()),
        enemy_thunderbots_config->getNavigatorConfig());
}

void SimulatedErForceSimTacticTestFixture::setTactic(
    std::shared_ptr<Tactic> friendly_tactic, std::shared_ptr<Tactic> enemy_tactic)
{
    setFriendlyTactic(friendly_tactic);
    setEnemyTactic(enemy_tactic);
}

void SimulatedErForceSimTacticTestFixture::setFriendlyTactic(
    std::shared_ptr<Tactic> tactic)
{
    if (tactic)
    {
        this->friendly_tactic = tactic;
    }
    else
    {
        LOG(FATAL) << "Friendly tactic is invalid" << std::endl;
    }
}

void SimulatedErForceSimTacticTestFixture::setEnemyTactic(std::shared_ptr<Tactic> tactic)
{
    if (tactic)
    {
        this->enemy_tactic = tactic;
    }
    else
    {
        LOG(FATAL) << "Enemy tactic is invalid" << std::endl;
    }
}

void SimulatedErForceSimTacticTestFixture::setFriendlyRobotId(RobotId friendly_robot_id)
{
    this->friendly_robot_id = friendly_robot_id;
}

void SimulatedErForceSimTacticTestFixture::setBothRobotId(RobotId friendly_robot_id,
                                                          RobotId enemy_robot_id)
{
    this->friendly_robot_id = friendly_robot_id;
    this->enemy_robot_id    = enemy_robot_id;
}

void SimulatedErForceSimTacticTestFixture::setMotionConstraints(
    const std::set<MotionConstraint>& friendly_motion_constraints,
    const std::set<MotionConstraint>& enemy_motion_constraints)
{
    this->friendly_motion_constraints = friendly_motion_constraints;
    this->enemy_motion_constraints    = enemy_motion_constraints;
}

void SimulatedErForceSimTacticTestFixture::updatePrimitives(
    const World& friendly_world, const World& enemy_world,
    std::shared_ptr<ErForceSimulator> simulator_to_update)
{
    updateFriendlyPrimitives(friendly_world, simulator_to_update);
    if (this->enemy_robot_id)
    {
        updateEnemyPrimitives(enemy_world, simulator_to_update);
    }
}

void SimulatedErForceSimTacticTestFixture::updateFriendlyPrimitives(
    const World& world, std::shared_ptr<ErForceSimulator> simulator_to_update)
{
    std::vector<std::unique_ptr<Intent>> intents;
    auto start_tick_time = std::chrono::system_clock::now();

    if (!friendly_robot_id)
    {
        LOG(FATAL) << "No friendly robot id set" << std::endl;
    }
    else if (auto new_robot = world.friendlyTeam().getRobotById(*friendly_robot_id))
    {
        auto intent = friendly_tactic->get(
            *world.friendlyTeam().getRobotById(*friendly_robot_id), world);
        intent->setMotionConstraints(friendly_motion_constraints);
        intents.push_back(std::move(intent));
    }
    else
    {
        LOG(FATAL) << "No friendly robot with robot id " << *friendly_robot_id
                   << std::endl;
    }

    auto primitive_set_msg = friendly_navigator->getAssignedPrimitives(world, intents);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerFriendlyTickTime(duration_ms);
    auto vision_msg = createVision(world);
    simulator_to_update->setYellowRobotPrimitiveSet(*primitive_set_msg,
                                                    std::move(vision_msg));
}

void SimulatedErForceSimTacticTestFixture::updateEnemyPrimitives(
    const World& world, std::shared_ptr<ErForceSimulator> simulator_to_update)
{
    std::vector<std::unique_ptr<Intent>> intents;
    auto start_tick_time = std::chrono::system_clock::now();

    if (!enemy_robot_id)
    {
        LOG(FATAL) << "No enemy robot id set" << std::endl;
    }
    else if (auto new_robot = world.friendlyTeam().getRobotById(*enemy_robot_id))
    {
        auto intent =
            enemy_tactic->get(*world.friendlyTeam().getRobotById(*enemy_robot_id), world);
        intent->setMotionConstraints(enemy_motion_constraints);
        intents.push_back(std::move(intent));
    }
    else
    {
        LOG(FATAL) << "No enemy robot with robot id " << *enemy_robot_id << std::endl;
    }

    auto primitive_set_msg = enemy_navigator->getAssignedPrimitives(world, intents);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerEnemyTickTime(duration_ms);
    auto defending_side = DefendingSideProto();
    defending_side.set_defending_side(
        DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_POS_X);
    auto vision_msg = createVision(world);
    simulator_to_update->setBlueRobotPrimitiveSet(*primitive_set_msg,
                                                  std::move(vision_msg));
}

std::optional<TbotsProto::PlayInfo> SimulatedErForceSimTacticTestFixture::getPlayInfo()
{
    return std::nullopt;
}

AIDrawFunction SimulatedErForceSimTacticTestFixture::getDrawFunctions()
{
    return drawNavigator(friendly_navigator);
}
