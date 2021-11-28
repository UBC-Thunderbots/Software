#include "software/simulated_tests/simulated_er_force_sim_play_test_fixture.h"

#include "proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/gui/drawing/navigator.h"
#include "software/test_util/test_util.h"

SimulatedErForceSimPlayTestFixture::SimulatedErForceSimPlayTestFixture()
    : ai_config(mutable_thunderbots_config->getMutableAiConfig()),
      ai_control_config(mutable_thunderbots_config->getMutableAiControlConfig()),
      sensor_fusion_config(mutable_thunderbots_config->getMutableSensorFusionConfig()),
      game_state(),
      ai(thunderbots_config->getAiConfig(), thunderbots_config->getAiControlConfig(),
         thunderbots_config->getPlayConfig())
{
}

void SimulatedErForceSimPlayTestFixture::SetUp()
{
    SimulatedErForceSimTestFixture::SetUp();

    ai_config            = mutable_thunderbots_config->getMutableAiConfig();
    ai_control_config    = mutable_thunderbots_config->getMutableAiControlConfig();
    sensor_fusion_config = mutable_thunderbots_config->getMutableSensorFusionConfig();

    ai = AI(thunderbots_config->getAiConfig(), thunderbots_config->getAiControlConfig(),
            thunderbots_config->getPlayConfig());
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
    ai_control_config->getMutableOverrideAiPlay()->setValue(true);
    ai_control_config->getMutableCurrentAiPlay()->setValue(ai_play);
}

void SimulatedErForceSimPlayTestFixture::setAIPlayConstructor(
    std::function<std::unique_ptr<Play>()> play_constructor)
{
    ai.overridePlayConstructor(play_constructor);
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
    const World& world, std::shared_ptr<ErForceSimulator> simulator_to_update)
{
    auto world_with_updated_game_state = world;
    world_with_updated_game_state.updateGameState(game_state);

    auto start_tick_time = std::chrono::system_clock::now();

    auto primitive_set_msg = ai.getPrimitives(world_with_updated_game_state);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerTickTime(duration_ms);
    auto vision_msg = createVision(world_with_updated_game_state);
    simulator_to_update->setYellowRobotPrimitiveSet(
        createNanoPbPrimitiveSet(*primitive_set_msg), std::move(vision_msg));
}

std::optional<PlayInfo> SimulatedErForceSimPlayTestFixture::getPlayInfo()
{
    return ai.getPlayInfo();
}

AIDrawFunction SimulatedErForceSimPlayTestFixture::getDrawFunctions()
{
    return drawNavigator(ai.getNavigator());
}
