#include "software/simulated_tests/simulated_tactic_test_fixture.h"

#include "proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"
#include "software/gui/drawing/navigator.h"
#include "software/simulation/simulator.h"
#include "software/test_util/test_util.h"

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

void SimulatedTacticTestFixture::setTactic(std::shared_ptr<Tactic> friendly_tactic,
                                           std::shared_ptr<Tactic> enemy_tactic)
{
    setFriendlyTactic(friendly_tactic);
    setEnemyTactic(enemy_tactic);
}

void SimulatedTacticTestFixture::setFriendlyTactic(std::shared_ptr<Tactic> tactic)
{
    CHECK(static_cast<bool>(tactic)) << "Friendly tactic is invalid" << std::endl;
    this->friendly_tactic = tactic;
}

void SimulatedTacticTestFixture::setEnemyTactic(std::shared_ptr<Tactic> tactic)
{
    CHECK(static_cast<bool>(tactic)) << "Enemy tactic is invalid" << std::endl;
    this->enemy_tactic = tactic;
}

void SimulatedTacticTestFixture::setFriendlyRobotId(RobotId friendly_robot_id)
{
    this->friendly_robot_id_opt = friendly_robot_id;
}

void SimulatedTacticTestFixture::setBothRobotId(RobotId friendly_robot_id,
                                                RobotId enemy_robot_id)
{
    this->friendly_robot_id_opt = friendly_robot_id;
    this->enemy_robot_id_opt    = enemy_robot_id;
}

void SimulatedTacticTestFixture::setMotionConstraints(
    const std::set<MotionConstraint>& friendly_motion_constraints,
    const std::set<MotionConstraint>& enemy_motion_constraints)
{
    this->friendly_motion_constraints = friendly_motion_constraints;
    this->enemy_motion_constraints    = enemy_motion_constraints;
}

void SimulatedTacticTestFixture::updatePrimitives(
    const World& friendly_world, const World& enemy_world,
    std::shared_ptr<Simulator> simulator_to_update)
{
    updateFriendlyPrimitives(friendly_world, simulator_to_update);
    if (this->enemy_robot_id_opt.has_value())
    {
        updateEnemyPrimitives(enemy_world, simulator_to_update);
    }
}

void SimulatedTacticTestFixture::updateFriendlyPrimitives(
    const World& world, std::shared_ptr<Simulator> simulator_to_update)
{
    std::vector<std::unique_ptr<Intent>> intents;
    auto start_tick_time = std::chrono::system_clock::now();

    CHECK(friendly_robot_id_opt.has_value()) << "No friendly robot id set" << std::endl;
    auto new_robot_opt = world.friendlyTeam().getRobotById(friendly_robot_id_opt.value());

    CHECK(new_robot_opt.has_value()) << "No friendly robot with robot id "
                                     << friendly_robot_id_opt.value() << std::endl;
    auto intent = friendly_tactic->get(new_robot_opt.value(), world);
    intent->setMotionConstraints(friendly_motion_constraints);
    intents.push_back(std::move(intent));

    auto primitive_set_msg = friendly_navigator->getAssignedPrimitives(world, intents);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerFriendlyTickTime(duration_ms);
    simulator_to_update->setYellowRobotPrimitiveSet(
        createNanoPbPrimitiveSet(*primitive_set_msg));
}

void SimulatedTacticTestFixture::updateEnemyPrimitives(
    const World& world, std::shared_ptr<Simulator> simulator_to_update)
{
    std::vector<std::unique_ptr<Intent>> intents;
    auto start_tick_time = std::chrono::system_clock::now();

    CHECK(enemy_robot_id_opt.has_value()) << "No enemy robot id set" << std::endl;
    auto new_robot_opt = world.friendlyTeam().getRobotById(enemy_robot_id_opt.value());

    CHECK(new_robot_opt.has_value())
        << "No enemy robot with robot id " << enemy_robot_id_opt.value() << std::endl;
    auto intent = enemy_tactic->get(new_robot_opt.value(), world);
    intent->setMotionConstraints(enemy_motion_constraints);
    intents.push_back(std::move(intent));

    auto primitive_set_msg = enemy_navigator->getAssignedPrimitives(world, intents);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerEnemyTickTime(duration_ms);
    auto defending_side = DefendingSideProto();
    defending_side.set_defending_side(
        DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_POS_X);
    simulator_to_update->setBlueTeamDefendingSide(defending_side);
    simulator_to_update->setBlueRobotPrimitiveSet(
        createNanoPbPrimitiveSet(*primitive_set_msg));
}

std::optional<TbotsProto::PlayInfo> SimulatedTacticTestFixture::getPlayInfo()
{
    TbotsProto::PlayInfo info;
    info.mutable_play()->set_play_name("SimulatedTacticTest");
    TbotsProto::PlayInfo_Tactic tactic_msg;
    tactic_msg.set_tactic_name(objectTypeName(*friendly_tactic));
    tactic_msg.set_tactic_fsm_state(friendly_tactic->getFSMState());
    CHECK(friendly_robot_id_opt.has_value()) << "No friendly robot id set" << std::endl;
    (*info.mutable_robot_tactic_assignment())[friendly_robot_id_opt.value()] = tactic_msg;
    return info;
}

AIDrawFunction SimulatedTacticTestFixture::getDrawFunctions()
{
    return drawNavigator(friendly_navigator);
}
