#include "software/simulated_tests/simulated_tactic_test_fixture.h"

#include "proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"
#include "software/gui/drawing/navigator.h"
#include "software/test_util/test_util.h"

SimulatedTacticTestFixture::SimulatedTacticTestFixture()
    : motion_constraints(),
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

void SimulatedTacticTestFixture::SetUp()
{
    SimulatedTestFixture::SetUp();
    navigator = std::make_shared<Navigator>(
        std::make_unique<VelocityObstaclePathManager>(
            std::make_unique<ThetaStarPathPlanner>(),
            RobotNavigationObstacleFactory(
                thunderbots_config->getRobotNavigationObstacleConfig())),
        RobotNavigationObstacleFactory(
            thunderbots_config->getRobotNavigationObstacleConfig()),
        thunderbots_config->getNavigatorConfig());
}

void SimulatedTacticTestFixture::setTactic(std::shared_ptr<Tactic> tactic)
{
    if (tactic)
    {
        this->tactic = tactic;
    }
    else
    {
        throw std::invalid_argument("Tactic is invalid");
    }
}

void SimulatedTacticTestFixture::setRobotId(RobotId robot_id)
{
    this->robot_id = robot_id;
}

void SimulatedTacticTestFixture::setMotionConstraints(
    const std::set<MotionConstraint>& motion_constraints)
{
    this->motion_constraints = motion_constraints;
}

void SimulatedTacticTestFixture::updatePrimitives(
    const World& world, std::shared_ptr<Simulator> simulator_to_update)
{
    std::vector<std::unique_ptr<Intent>> intents;
    auto start_tick_time = std::chrono::system_clock::now();

    if (!robot_id)
    {
        LOG(FATAL) << "No robot id set" << std::endl;
    }
    else if (auto new_robot = world.friendlyTeam().getRobotById(*robot_id))
    {
        auto intent = tactic->get(*world.friendlyTeam().getRobotById(*robot_id), world);
        intent->setMotionConstraints(motion_constraints);
        intents.push_back(std::move(intent));
    }
    else
    {
        LOG(FATAL) << "No robot with robot id " << *robot_id << std::endl;
    }

    auto primitive_set_msg = navigator->getAssignedPrimitives(world, intents);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerTickTime(duration_ms);
    simulator_to_update->setYellowRobotPrimitiveSet(
        createNanoPbPrimitiveSet(*primitive_set_msg));
}


std::optional<PlayInfo> SimulatedTacticTestFixture::getPlayInfo()
{
    return std::nullopt;
}

AIDrawFunction SimulatedTacticTestFixture::getDrawFunctions()
{
    return drawNavigator(navigator);
}
