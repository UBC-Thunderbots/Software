#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/hl/stp/play/assigned_tactics_play.h"
#include "software/test_util/test_util.h"

SimulatedErForceSimPlayTestFixture::SimulatedErForceSimPlayTestFixture()
    : ai_config(friendly_mutable_thunderbots_config->getMutableAiConfig()),
      sensor_fusion_config(
          friendly_mutable_thunderbots_config->getMutableSensorFusionConfig()),
      game_state(),
      ai(friendly_thunderbots_config->getAiConfig())
{
}

void SimulatedErForceSimPlayTestFixture::SetUp()
{
    SimulatedErForceSimTestFixture::SetUp();

    ai_config = friendly_mutable_thunderbots_config->getMutableAiConfig();
    sensor_fusion_config =
        friendly_mutable_thunderbots_config->getMutableSensorFusionConfig();

    ai = AI(friendly_thunderbots_config->getAiConfig());
}

void SimulatedErForceSimPlayTestFixture::setFriendlyGoalie(RobotId goalie_id)
{
    sensor_fusion_config->getMutableFriendlyGoalieId()->setValue(
        static_cast<int>(goalie_id));
}

void SimulatedErForceSimPlayTestFixture::setEnemyGoalie(RobotId goalie_id)
{
    sensor_fusion_config->getMutableEnemyGoalieId()->setValue(
        static_cast<int>(goalie_id));
}

void SimulatedErForceSimPlayTestFixture::setAIPlay(const std::string& ai_play)
{
    ai_config->getMutableAiControlConfig()->getMutableOverrideAiPlay()->setValue(true);
    ai_config->getMutableAiControlConfig()->getMutableCurrentAiPlay()->setValue(ai_play);
}

void SimulatedErForceSimPlayTestFixture::setAIPlay(std::unique_ptr<Play> play)
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
    ai_config->getMutableAiControlConfig()->getMutableOverrideAiPlay()->setValue(false);
    CHECK(static_cast<bool>(tactic)) << "Tactic is invalid" << std::endl;
    std::unique_ptr<AssignedTacticsPlay> play =
        std::make_unique<AssignedTacticsPlay>(ai_config);
    std::map<RobotId, std::set<TbotsProto::MotionConstraint>>
        motion_constraint_override_map;
    motion_constraint_override_map[id] = motion_constraints;
    play->updateControlParams({{id, tactic}});
    setAIPlay(std::move(play));
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
    auto world_with_updated_game_state = friendly_world;
    world_with_updated_game_state.updateGameState(game_state);

    auto start_tick_time = std::chrono::system_clock::now();

    auto primitive_set_msg = ai.getPrimitives(world_with_updated_game_state);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerFriendlyTickTime(duration_ms);
    auto world_msg = createWorld(world_with_updated_game_state);
    simulator_to_update->setYellowRobotPrimitiveSet(*primitive_set_msg,
                                                    std::move(world_msg));
}

const std::shared_ptr<AiConfig> SimulatedErForceSimPlayTestFixture::getAiConfig() const
{
    return ai_config;
}

std::optional<TbotsProto::PlayInfo> SimulatedErForceSimPlayTestFixture::getPlayInfo()
{
    return ai.getPlayInfo();
}

AIDrawFunction SimulatedErForceSimPlayTestFixture::getDrawFunctions()
{
    return AIDrawFunction([](QGraphicsScene* scene) {});
}
