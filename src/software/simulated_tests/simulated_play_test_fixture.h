#pragma once

#include <gtest/gtest.h>

#include "software/ai/ai.h"
#include "software/simulated_tests/simulated_test_fixture.h"

/**
 * This is a test fixture designed to make it easy to write integration tests. It provides
 * an easy interface to set up robots on the field, and then validate how the world
 * changes over time during simulation. This allows us to easily write tests for
 * the AI's behaviour.
 */
class SimulatedPlayTestFixture : public SimulatedTestFixture
{
   public:
    explicit SimulatedPlayTestFixture();

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

    void updatePrimitives(const World& world,
                          std::shared_ptr<Simulator> simulator_to_update) override;
    std::optional<PlayInfo> getPlayInfo() override;
    AIDrawFunction getDrawFunctions() override;

   private:
    // The AI being tested and used in simulation
    AI ai;
};
