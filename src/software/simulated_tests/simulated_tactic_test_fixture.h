#pragma once

#include <gtest/gtest.h>

#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/navigator/navigator.h"
#include "software/simulated_tests/simulated_test_fixture.h"

/**
 * This is a test fixture designed to make it easy to write integration tests for Tactics.
 * It provides an easy interface to set up a single robot running a single tactic , and
 * then validate how the world changes over time during simulation
 *
 * Since users can't update the Tactic in sync with simulation ticks, the
 * SimulatedTacticTestFixture will update the robot of the tactic
 */
class SimulatedTacticTestFixture : public SimulatedTestFixture
{
   public:
    explicit SimulatedTacticTestFixture();

   protected:
    void SetUp() override;

    /**
     * Sets the friendly and enemy tactic to the given tactics, otherwise default to the
     * halt tactic
     *
     * @param friendly_tactic The friendly tactic
     * @param enemy_tactic The enemy tactic
     *
     * @throw invalid_argument if any tactic is invalid
     */
    void setTactic(
        std::shared_ptr<Tactic> friendly_tactic = std::make_shared<StopTactic>(true),
        std::shared_ptr<Tactic> enemy_tactic    = std::make_shared<StopTactic>(true));

    /**
     * Sets the friendly tactic to test
     *
     * @param tactic The tactic under test
     *
     * @throw invalid_argument if does not contain an tactic
     */
    void setFriendlyTactic(std::shared_ptr<Tactic> tactic);

    /**
     * Sets the tactic for the enemy
     *
     * @param tactic The tactic under test (for the enemy)
     *
     * @throw invalid_argument if does not contain a tactic
     */
    void setEnemyTactic(std::shared_ptr<Tactic> tactic);

    /**
     * Sets the friendly and enemy robot IDs
     *
     * @param friendly_robot_id The friendly robot ID
     * @param enemy_robot_id The enemy robot ID, defaults to UINT_MAX as a flag to not set
     * an enemy robot ID
     */
    void setRobotId(RobotId friendly_robot_id, RobotId enemy_robot_id = UINT_MAX);

    /**
     * Sets the motion constraints for the friendly and enemy (if applicable) teams
     *
     * @param friendly_motion_constraints The friendly motion constraint
     * @param enemy_motion_constraints The enemy motion constraint
     */
    void setMotionConstraints(
        const std::set<MotionConstraint>& friendly_motion_constraints,
        const std::set<MotionConstraint>& enemy_motion_constraints = {});

   private:
    void updatePrimitives(const World& friendly_world, const World& enemy_world,
                          std::shared_ptr<Simulator> simulator_to_update) override;
    void updateFriendlyPrimitives(const World& world,
                                  std::shared_ptr<Simulator> simulator_to_update);
    void updateEnemyPrimitives(const World& world,
                               std::shared_ptr<Simulator> simulator_to_update);

    std::optional<PlayInfo> getPlayInfo() override;
    AIDrawFunction getDrawFunctions() override;

    std::shared_ptr<Tactic> friendly_tactic;
    std::shared_ptr<Tactic> enemy_tactic;
    std::optional<RobotId> friendly_robot_id;
    std::optional<RobotId> enemy_robot_id;

    // Motion constraints to set for intent
    std::set<MotionConstraint> friendly_motion_constraints;
    std::set<MotionConstraint> enemy_motion_constraints;
    // The Navigator
    std::shared_ptr<Navigator> friendly_navigator;
    std::shared_ptr<Navigator> enemy_navigator;
};
