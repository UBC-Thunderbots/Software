#include "software/simulated_tests/simulated_play_test_fixture.h"

#include "proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "software/gui/drawing/navigator.h"
#include "software/test_util/test_util.h"

SimulatedPlayTestFixture::SimulatedPlayTestFixture()
    : friendly_ai_config(friendly_mutable_thunderbots_config->getMutableAiConfig()),
      friendly_ai_control_config(friendly_mutable_thunderbots_config->getMutableAiControlConfig()),
      friendly_sensor_fusion_config(
          friendly_mutable_thunderbots_config->getMutableSensorFusionConfig()),
      game_state(),
      friendly_ai(friendly_thunderbots_config->getAiConfig(),
         friendly_thunderbots_config->getAiControlConfig(),
         friendly_thunderbots_config->getPlayConfig())
      enemy_ai_config(enemy_mutable_thunderbots_config->getMutableAiConfig()),
      enemy_ai_control_config(enemy_mutable_thunderbots_config->getMutableAiControlConfig()),
      enemy_sensor_fusion_config(
          enemy_mutable_thunderbots_config->getMutableSensorFusionConfig()),
      game_state(),
      enemy_ai(enemy_thunderbots_config->getAiConfig(),
         enemy_thunderbots_config->getAiControlConfig(),
         enemy_thunderbots_config->getPlayConfig())
{
}

void SimulatedPlayTestFixture::SetUp()
{
    SimulatedTestFixture::SetUp();
    
    // Friendly Config
    friendly_ai_config         = friendly_mutable_thunderbots_config->getMutableAiConfig();
    friendly_ai_control_config = friendly_mutable_thunderbots_config->getMutableAiControlConfig();
    friendly_sensor_fusion_config =
        friendly_mutable_thunderbots_config->getMutableSensorFusionConfig();

    friendly_ai = AI(friendly_thunderbots_config->getAiConfig(),
                    friendly_thunderbots_config->getAiControlConfig(),
                    friendly_thunderbots_config->getPlayConfig());

    // Enemy Config
    enemy_ai_config             = enemy_mutable_thunderbots_config->getMutableAiConfig();
    enemy_ai_control_config     = enemy_mutable_thunderbots_config->getMutableAiControlConfig();
    enemy_sensor_fusion_config =
        enemy_mutable_thunderbots_config->getMutableSensorFusionConfig();
    enemy_ai = AI(enemy_thunderbots_config->getAiConfig(),
                  enemy_thunderbots_config->getAiControlConfig(),
                  enemy_thunderbots_config->getPlayConfig());
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

void SimulatedPlayTestFixture::setFriendlyAIPlay(const std::string& friendly_ai_play)
{
    friendly_ai_control_config->getMutableOverrideAiPlay()->setValue(true);
    friendly_ai_control_config->getMutableCurrentAiPlay()->setValue(friendly_ai_play);
}

void SimulatedPlayTestFixture::setEnemyAIPlay(const std::string& enemy_ai_play)
{
    enemy_ai_control_config->getMutableOverrideAiPlay()->setValue(true);
    enemy_ai_control_config->getMutableCurrentAiPlay()->setValue(enemy_ai_play);
}

void SimulatedPlayTestFixture::setFriendlyAIPlayConstructor(
    std::function<std::unique_ptr<Play>()> friendly_play_constructor)
{
    friendly_ai.overridePlayConstructor(friendly_play_constructor);
}

void SimulatredPlayTestFixture::setEnemyAIPlayConstructor(
    std::function<std::unique_ptr<Play>()> enemy_play_constructor)
{
    enemy_ai.overridePlayConstructor(enemy_play_constructor);
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
    const World& friendly_world, const World& enemy_world,
    std::shared_ptr<Simulator> simulator_to_update)
{
    updateFriendlyPrimitives(friendly_world,simulator_to_update);
    updateEnemyPrimitives(enemy_world,simulator_to_update);
}

void SimulatedPlayTestFixture::updateFriendlyPrimitives(
        const World& friendly_world, 
        std::shared_ptr<Simulator> simulator_to_update)
{
    //TODO: Check this - at the moment its copy pasta'd from the old update prim
    auto world_with_updated_game_state = friendly_world;
    world_with_updated_game_state.updateGameState(game_state);

    auto start_tick_time = std::chrono::system_clock::now();

    auto primitive_set_msg = friendly_ai.getPrimitives(world_with_updated_game_state);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerFriendlyTickTime(duration_ms);
    simulator_to_update->setYellowRobotPrimitiveSet(
        createNanoPbPrimitiveSet(*primitive_set_msg));
}

void SimulatedPlayTestFixture::updateEnemyPrimitives(
        const World& enemy_world,
        std::shared_ptr<Simulator> simulator_to_update)
{
    auto world_with_updated_game_state = enemy_world;
    world_with_updated_game_state.updateGameState(game_state);

    auto start_tick_time = std::chrono::system_clock::now();

    auto primitive_set_msg = enemy_ai.getPrimitives(world_with_updated_game_state);
    double duration_ms     = ::TestUtil::millisecondsSince(start_tick_time);
    registerEnemyTickTime(duration_ms);
    simulator_to_update->setBlueRobotPrimitiveSet(
        createNanoPbPrimitiveSet(*primitive_set_msg));
}

const std::shared_ptr<AiConfig> SimulatedPlayTestFixture::getFriendlyAiConfig() const
{
    return friendly_ai_config;
}

const std::shared_ptr<AiConfig> SimulatedPlayTestFixture::getEnemyAiConfig() const
{
    return enemy_ai_config;
}

std::optional<TbotsProto::PlayInfo> SimulatedPlayTestFixture::getFriendlyPlayInfo()
{
    return friendly_ai.getPlayInfo();
}

std::optional<TbotsProto::PlayInfo> SimulatedPlayTestFixture::getEnemyPlayInfo()
{
    return enemy_ai.getPlayInfo();
}

AIDrawFunction SimulatedPlayTestFixture::getDrawFunctions()
{
    return drawNavigator(ai.getNavigator());
}
