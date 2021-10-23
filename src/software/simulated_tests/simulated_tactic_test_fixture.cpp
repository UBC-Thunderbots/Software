#include "software/simulated_tests/simulated_tactic_test_fixture.h"

#include "proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"
#include "software/gui/drawing/navigator.h"
#include "software/test_util/test_util.h"
#include "software/simulation/simulator.h"

SimulatedTacticTestFixture::SimulatedTacticTestFixture()
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

void SimulatedTacticTestFixture::SetUp()
{
    SimulatedTestFixture::SetUp();
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

void SimulatedTacticTestFixture::setFriendlyTactic(std::shared_ptr<Tactic> tactic)
{
    if (tactic)
    {
        this->friendly_tactic = tactic;
    }
    else
    {
        throw std::invalid_argument("Tactic is invalid");
    }
}

void SimulatedTacticTestFixture::setEnemyTactic(std::shared_ptr<Tactic> tactic)
{
    if (tactic)
    {
        this->enemy_tactic = tactic;
    }
    else
    {
        throw std::invalid_argument("Tactic is invalid");
    }
}

void SimulatedTacticTestFixture::setFriendlyRobotId(RobotId robot_id)
{
    this->friendly_robot_id = robot_id;
}

void SimulatedTacticTestFixture::setEnemyRobotId(RobotId robot_id)
{
    this->enemy_robot_id = robot_id;
}

void SimulatedTacticTestFixture::setFriendlyMotionConstraints(
    const std::set<MotionConstraint>& motion_constraints)
{
    this->friendly_motion_constraints = motion_constraints;
}

void SimulatedTacticTestFixture::setEnemyMotionConstraints(
    const std::set<MotionConstraint>& motion_constraints)
{
    this->enemy_motion_constraints = motion_constraints;
}

void SimulatedTacticTestFixture::updatePrimitives(
    const World& friendly_world, const World& enemy_world, std::shared_ptr<Simulator> simulator_to_update)
{
    updateFriendlyPrimitives(friendly_world, simulator_to_update);
    updateEnemyPrimitives(enemy_world, simulator_to_update);
}

void SimulatedTacticTestFixture::updateFriendlyPrimitives(
    const World& world, std::shared_ptr<Simulator> simulator_to_update)
{
    std::vector<std::unique_ptr<Intent>> intents;
    auto start_tick_time = std::chrono::system_clock::now();

    if (!friendly_robot_id)
    {
        LOG(FATAL) << "No friendly robot id set" << std::endl;
    }
    else if (auto new_robot = world.friendlyTeam().getRobotById(*friendly_robot_id))
    {
        auto intent = friendly_tactic->get(*world.friendlyTeam().getRobotById(*friendly_robot_id), world);
        intent->setMotionConstraints(friendly_motion_constraints);
        intents.push_back(std::move(intent));
    }
    else
    {
        LOG(FATAL) << "No friendly robot with robot id " << *friendly_robot_id << std::endl;
    }

    auto primitive_set_msg = friendly_navigator->getAssignedPrimitives(world, intents);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerTickTime(duration_ms);
    simulator_to_update->setYellowRobotPrimitiveSet(
        createNanoPbPrimitiveSet(*primitive_set_msg));
}

void SimulatedTacticTestFixture::updateEnemyPrimitives(
    const World& world, std::shared_ptr<Simulator> simulator_to_update)
{
    std::vector<std::unique_ptr<Intent>> intents;
    auto start_tick_time = std::chrono::system_clock::now();

    if (!enemy_robot_id)
    {
        LOG(FATAL) << "No enemy robot id set" << std::endl;
    }
    else if (auto new_robot = world.friendlyTeam().getRobotById(*enemy_robot_id))
    {
        auto intent = enemy_tactic->get(*world.friendlyTeam().getRobotById(*enemy_robot_id), world);
        intent->setMotionConstraints(enemy_motion_constraints);
        intents.push_back(std::move(intent));
    }
    else
    {
        LOG(FATAL) << "No enemy robot with robot id " << *enemy_robot_id << std::endl;
    }

    auto primitive_set_msg = enemy_navigator->getAssignedPrimitives(world, intents);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerTickTime(duration_ms);
    // auto defending_side = DefendingSideProto();
    // defending_side.set_defending_side(DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_NEG_X);
    // simulator_to_update->setBlueTeamDefendingSide(defending_side);
    simulator_to_update->setBlueRobotPrimitiveSet(
        createNanoPbPrimitiveSet(*primitive_set_msg));
}

std::optional<PlayInfo> SimulatedTacticTestFixture::getPlayInfo()
{
    return std::nullopt;
}

AIDrawFunction SimulatedTacticTestFixture::getDrawFunctions()
{
    return drawNavigator(friendly_navigator);
}
