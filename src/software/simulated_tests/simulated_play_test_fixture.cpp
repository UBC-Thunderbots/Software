#include "software/simulated_tests/simulated_play_test_fixture.h"

#include "proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "software/gui/drawing/navigator.h"
#include "software/test_util/test_util.h"

SimulatedPlayTestFixture::SimulatedPlayTestFixture()
    : ai_config(friendly_mutable_thunderbots_config->getMutableAiConfig()),
      sensor_fusion_config(
          friendly_mutable_thunderbots_config->getMutableSensorFusionConfig()),
      game_state(),
      ai(friendly_thunderbots_config->getAiConfig())
{
}

void SimulatedPlayTestFixture::SetUp()
{
    SimulatedTestFixture::SetUp();

    ai_config = friendly_mutable_thunderbots_config->getMutableAiConfig();
    sensor_fusion_config =
        friendly_mutable_thunderbots_config->getMutableSensorFusionConfig();

    ai = AI(friendly_thunderbots_config->getAiConfig());
}

void SimulatedPlayTestFixture::setFriendlyGoalie(RobotId goalie_id)
{
    sensor_fusion_config->getMutableFriendlyGoalieId()->setValue(
        static_cast<int>(goalie_id));
}

void SimulatedPlayTestFixture::setEnemyGoalie(RobotId goalie_id)
{
    sensor_fusion_config->getMutableEnemyGoalieId()->setValue(
        static_cast<int>(goalie_id));
}

void SimulatedPlayTestFixture::setAIPlay(const std::string& ai_play)
{
    ai_config->getMutableAiControlConfig()->getMutableOverrideAiPlay()->setValue(true);
    ai_config->getMutableAiControlConfig()->getMutableCurrentAiPlay()->setValue(ai_play);
}

void SimulatedPlayTestFixture::setAIPlay(std::unique_ptr<Play> play)
{
    ai.overridePlay(std::move(play));
}

void SimulatedPlayTestFixture::setRefereeCommand(
    const RefereeCommand& current_referee_command,
    const RefereeCommand& previous_referee_command)
{
    game_state.updateRefereeCommand(previous_referee_command);
    game_state.updateRefereeCommand(current_referee_command);
}

void SimulatedPlayTestFixture::setGameState(const GameState& game_state_)
{
    game_state = game_state_;
}

void SimulatedPlayTestFixture::updatePrimitives(
    const World& friendly_world, const World& unused_enemy_world,
    std::shared_ptr<Simulator> simulator_to_update)
{
    auto world_with_updated_game_state = friendly_world;
    world_with_updated_game_state.updateGameState(game_state);

    auto start_tick_time = std::chrono::system_clock::now();

    auto primitive_set_msg = ai.getPrimitives(world_with_updated_game_state);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerFriendlyTickTime(duration_ms);
    simulator_to_update->setYellowRobotPrimitiveSet(
        createNanoPbPrimitiveSet(*primitive_set_msg));
}

const std::shared_ptr<AiConfig> SimulatedPlayTestFixture::getAiConfig() const
{
    return ai_config;
}

std::optional<TbotsProto::PlayInfo> SimulatedPlayTestFixture::getPlayInfo()
{
    return ai.getPlayInfo();
}

AIDrawFunction SimulatedPlayTestFixture::getDrawFunctions()
{
    return AIDrawFunction([](QGraphicsScene* scene) {
    });
}
