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
     * @param ai_play The AI play name enum
     */
    void setAiPlay(const TbotsProto::PlayName& ai_play_name);

    /**
     * Sets the tactic to the given tactic
     *
     * @param id the robot id of the robot to run the tactic on
     * @param tactic The friendly tactic
     * @param motion_constraints optionally override motion constraints
     *
     * @throw invalid_argument if any tactic is invalid
     */
    void setTactic(RobotId id, std::shared_ptr<Tactic> tactic);
    void setTactic(RobotId id, std::shared_ptr<Tactic> tactic,
                   std::set<TbotsProto::MotionConstraint> motion_constraints);

    /**
     * Sets the AI play to be used to run in the simulated test
     *
     * @param play_ The play
     */
    void setAiPlay(std::unique_ptr<Play> play);

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
    const TbotsProto::AiConfig getAiConfig() const;

    std::optional<TbotsProto::PlayInfo> getPlayInfo() override;

    std::shared_ptr<Strategy> strategy;

   private:
    void updatePrimitives(const World& friendly_world, const World& enemy_world,
                          std::shared_ptr<ErForceSimulator> simulator_to_update) override;

    GameState game_state;

    // The AI being tested and used in simulation
    Ai ai;
};
