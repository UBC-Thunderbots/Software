#pragma once

#include <gtest/gtest.h>

#include "software/ai/ai.h"
#include "software/simulated_tests/simulated_er_force_sim_test_fixture.h"

/**
 * This is a test fixture designed to make it easy to write integration tests. It provides
 * an easy interface to set up robots on the field, and then validate how the world
 * changes over time during simulation. This allows us to easily write tests for
 * the AI's behaviour.
 */
class SimulatedErForceSimPlayTestFixture : public SimulatedErForceSimTestFixture
{
   public:
    explicit SimulatedErForceSimPlayTestFixture();

   protected:
    void SetUp() override;

    /**
     * Sets the goalie for the specified team. If this function is not called,
     * the goalie will be set to the default ID of the DynamicParameters
     *
     * @param goalie_id The ID of the robot to be goalie
     */
    void setFriendlyGoalie(RobotId goalie_id);
    void setEnemyGoalie(RobotId goalie_id);

    /**
     * Sets the AI play to run in the simulated test
     *
     * @param ai_play The name of the AI play
     */
    void setAIPlay(const std::string& ai_play);

    /**
     * Sets the AI play constructor to be used to run in the simulated test
     *
     * @param play_constructor The constructor for the play
     */
    void setAIPlayConstructor(std::function<std::unique_ptr<Play>()> play_constructor);

    /**
     * Sets the Referee command to override for the simulated test
     *
     * @param current_referee_command The name of the current referee command to set
     * @param previous_referee_command The name of the previous referee command to set
     */
    void setRefereeCommand(const RefereeCommand& current_referee_command,
                           const RefereeCommand& previous_referee_command);

    /**
     * Sets the game state to use while testing
     *
     * @param game_state_ The game state to set
     */
    void setGameState(const GameState& game_state_);

    /**
     * Gets the configs used in simulation
     * Useful for constructing duplicates of Obstacle Factory
     *
     * @return the Ai Config
     */
    const std::shared_ptr<AiConfig> getAiConfig() const;

    std::optional<TbotsProto::PlayInfo> getPlayInfo() override;
    AIDrawFunction getDrawFunctions() override;

   private:
    void updatePrimitives(const World& world,
                          std::shared_ptr<ErForceSimulator> simulator_to_update) override;
    // The configs being used in simulation
    std::shared_ptr<AiConfig> ai_config;
    std::shared_ptr<AiControlConfig> ai_control_config;
    std::shared_ptr<SensorFusionConfig> sensor_fusion_config;

    GameState game_state;

    // The AI being tested and used in simulation
    AI ai;
};
