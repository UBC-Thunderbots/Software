#pragma once

#include <gtest/gtest.h>

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
     * Sets the friendly tactic to test
     *
     * @param tactic The tactic under test
     *
     * @throw invalid_argument if does not contain an tactic
     */
    void setFriendlyTactic(std::shared_ptr<Tactic> tactic);

    /**
     * Sets the tactic for the enemy (if applicable)
     * 
     * @param tactic The tactic under test (for the enemy)
     * 
     * @throw invalid_argument if does not contain a tactic
     */
    void setEnemyTactic(std::shared_ptr<Tactic> tactic);

    /**
     * Sets the friendly robot id to run tactic on
     *
     * @param robot_id The robot id to run tactic on
     */
    void setFriendlyRobotId(RobotId robot_id);

    /**
     * Sets the enemy robot id to run the enemy tactic on
     * 
     * @param robot_id The enemy robot id to run the enemy tactic on
     */
    void setEnemyRobotId(RobotId robot_id);

    /**
     * Sets the friendly motion constraints to test
     *
     * @param motion_constraints The motion constraints for the tactic
     */
    void setFriendlyMotionConstraints(const std::set<MotionConstraint>& motion_constraints);

    /**
     * Sets the enemy motion constraints for test
     * 
     * @param motion_constraints The motion constraints for the enemy tactic
     */
    void setEnemyMotionConstraints(const std::set<MotionConstraint>& motion_constriants);

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
