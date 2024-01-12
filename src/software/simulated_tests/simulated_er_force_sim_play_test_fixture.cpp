#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/hl/stp/play/assigned_tactics_play.h"
#include "software/test_util/test_util.h"

SimulatedErForceSimPlayTestFixture::SimulatedErForceSimPlayTestFixture()
    : game_state(), ai(friendly_thunderbots_config.ai_config()), strategy(std::make_shared<Strategy>(friendly_thunderbots_config.ai_config()))
{
}

void SimulatedErForceSimPlayTestFixture::SetUp()
{
    SimulatedErForceSimTestFixture::SetUp();
}

void SimulatedErForceSimPlayTestFixture::setFriendlyGoalie(RobotId goalie_id)
{
    friendly_thunderbots_config.mutable_sensor_fusion_config()->set_friendly_goalie_id(
        static_cast<int>(goalie_id));
}

void SimulatedErForceSimPlayTestFixture::setEnemyGoalie(RobotId goalie_id)
{
    friendly_thunderbots_config.mutable_sensor_fusion_config()->set_enemy_goalie_id(
        static_cast<int>(goalie_id));
}

void SimulatedErForceSimPlayTestFixture::setAiPlay(
    const TbotsProto::PlayName& ai_play_name)
{
    friendly_thunderbots_config.mutable_ai_config()
        ->mutable_ai_control_config()
        ->set_override_ai_play(ai_play_name);

    ai = Ai(friendly_thunderbots_config.ai_config());
}

void SimulatedErForceSimPlayTestFixture::setAiPlay(std::unique_ptr<Play> play)
{
    ai.overridePlay(std::move(play));
}

void SimulatedErForceSimPlayTestFixture::setTactic(RobotId id,
                                                   std::shared_ptr<Tactic> tactic)
{
    setTactic(id, tactic, {});
}

void SimulatedErForceSimPlayTestFixture::setTactic(
    RobotId id, std::shared_ptr<Tactic> tactic,
    std::set<TbotsProto::MotionConstraint> motion_constraints)
{
    CHECK(static_cast<bool>(tactic)) << "Tactic is invalid" << std::endl;
    std::unique_ptr<AssignedTacticsPlay> play =
        std::make_unique<AssignedTacticsPlay>(friendly_thunderbots_config.ai_config(), std::make_shared<Strategy>(friendly_thunderbots_config.ai_config()));
    std::map<RobotId, std::set<TbotsProto::MotionConstraint>>
        motion_constraint_override_map;
    motion_constraint_override_map[id] = motion_constraints;
    play->updateControlParams({{id, tactic}}, motion_constraint_override_map);
    setAiPlay(std::move(play));
}

void SimulatedErForceSimPlayTestFixture::setRefereeCommand(
    const RefereeCommand& current_referee_command,
    const RefereeCommand& previous_referee_command)
{
    game_state.updateRefereeCommand(previous_referee_command);
    game_state.updateRefereeCommand(current_referee_command);
}

void SimulatedErForceSimPlayTestFixture::setGameState(const GameState& game_state_)
{
    game_state = game_state_;
}

void SimulatedErForceSimPlayTestFixture::updatePrimitives(
    const World& friendly_world, const World&,
    std::shared_ptr<ErForceSimulator> simulator_to_update)
{
    strategy->updateWorld(friendly_world);

    auto world_with_updated_game_state = friendly_world;
    world_with_updated_game_state.updateGameState(game_state);

    auto start_tick_time = std::chrono::system_clock::now();

    auto primitive_set_msg = ai.getPrimitives(world_with_updated_game_state);
    LOG(VISUALIZE) << ai.getPlayInfo();
    LOG(VISUALIZE) << *primitive_set_msg;

    double duration_ms = ::TestUtil::millisecondsSince(start_tick_time);
    registerFriendlyTickTime(duration_ms);
    auto world_msg = createWorld(world_with_updated_game_state);
    simulator_to_update->setYellowRobotPrimitiveSet(*primitive_set_msg,
                                                    std::move(world_msg));
}

const TbotsProto::AiConfig SimulatedErForceSimPlayTestFixture::getAiConfig() const
{
    return friendly_thunderbots_config.ai_config();
}

std::optional<TbotsProto::PlayInfo> SimulatedErForceSimPlayTestFixture::getPlayInfo()
{
    return ai.getPlayInfo();
}

const std::shared_ptr<Strategy> SimulatedErForceSimPlayTestFixture::getStrategy() const
{
    return strategy;
}
