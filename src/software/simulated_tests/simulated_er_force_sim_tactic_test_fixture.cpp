#include "software/simulated_tests/simulated_er_force_sim_tactic_test_fixture.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"
#include "software/simulation/er_force_simulator.h"
#include "software/test_util/test_util.h"

SimulatedErForceSimTacticTestFixture::SimulatedErForceSimTacticTestFixture()
    : friendly_motion_constraints(),
      enemy_motion_constraints(),

      friendly_navigator(std::make_shared<Navigator>(
          std::make_unique<VelocityObstaclePathManager>(
              std::make_unique<ThetaStarPathPlanner>(), RobotNavigationObstacleFactory()),
          RobotNavigationObstacleFactory())),

      enemy_navigator(std::make_shared<Navigator>(
          std::make_unique<VelocityObstaclePathManager>(
              std::make_unique<ThetaStarPathPlanner>(), RobotNavigationObstacleFactory()),
          RobotNavigationObstacleFactory()))
{
}

void SimulatedErForceSimTacticTestFixture::SetUp()
{
    SimulatedErForceSimTestFixture::SetUp();
    friendly_navigator = std::make_shared<Navigator>(
        std::make_unique<VelocityObstaclePathManager>(
            std::make_unique<ThetaStarPathPlanner>(), RobotNavigationObstacleFactory()),
        RobotNavigationObstacleFactory());

    enemy_navigator = std::make_shared<Navigator>(
        std::make_unique<VelocityObstaclePathManager>(
            std::make_unique<ThetaStarPathPlanner>(), RobotNavigationObstacleFactory()),
        RobotNavigationObstacleFactory());
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
    CHECK(static_cast<bool>(tactic)) << "Friendly tactic is invalid" << std::endl;
    this->friendly_tactic = tactic;
}

void SimulatedErForceSimTacticTestFixture::setEnemyTactic(std::shared_ptr<Tactic> tactic)
{
    CHECK(static_cast<bool>(tactic)) << "Enemy tactic is invalid" << std::endl;
    this->enemy_tactic = tactic;
}

void SimulatedErForceSimTacticTestFixture::setFriendlyRobotId(RobotId friendly_robot_id)
{
    this->friendly_robot_id_opt = friendly_robot_id;
}

void SimulatedErForceSimTacticTestFixture::setBothRobotId(RobotId friendly_robot_id,
                                                          RobotId enemy_robot_id)
{
    this->friendly_robot_id_opt = friendly_robot_id;
    this->enemy_robot_id_opt    = enemy_robot_id;
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
    CHECK(friendly_robot_id_opt.has_value()) << "No friendly robot id set" << std::endl;
    registerFriendlyTickTime(
        updatePrimitives(friendly_world, simulator_to_update, friendly_navigator,
                         friendly_robot_id_opt.value(), friendly_tactic,
                         friendly_motion_constraints, friendly_thunderbots_config));
    if (this->enemy_robot_id_opt.has_value())
    {
        registerEnemyTickTime(updatePrimitives(
            enemy_world, simulator_to_update, enemy_navigator, enemy_robot_id_opt.value(),
            enemy_tactic, enemy_motion_constraints, enemy_thunderbots_config));
    }
}

double SimulatedErForceSimTacticTestFixture::updatePrimitives(
    const World& world, std::shared_ptr<ErForceSimulator> simulator_to_update,
    std::shared_ptr<Navigator> navigator, RobotId robot_id,
    std::shared_ptr<Tactic> tactic, const std::set<MotionConstraint>& motion_constraints,
    TbotsProto::ThunderbotsConfig config)
{
    std::vector<std::unique_ptr<Intent>> intents;
    auto start_tick_time = std::chrono::system_clock::now();

    auto new_robot_opt = world.friendlyTeam().getRobotById(robot_id);

    CHECK(new_robot_opt.has_value())
        << "No robot with robot id " << robot_id << std::endl;
    auto intent = tactic->get(new_robot_opt.value(), world);
    intent->setMotionConstraints(motion_constraints);
    intents.push_back(std::move(intent));

    auto primitive_set_msg = navigator->getAssignedPrimitives(world, intents);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    auto world_msg         = createWorld(world);

    if (config.sensor_fusion_config().friendly_color_yellow())
    {
        simulator_to_update->setYellowRobotPrimitiveSet(*primitive_set_msg,
                                                        std::move(world_msg));
    }
    else
    {
        simulator_to_update->setBlueRobotPrimitiveSet(*primitive_set_msg,
                                                      std::move(world_msg));
    }
    return duration_ms;
}

std::optional<TbotsProto::PlayInfo> SimulatedErForceSimTacticTestFixture::getPlayInfo()
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
