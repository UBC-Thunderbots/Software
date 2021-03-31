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
     * Sets the tactic to test
     *
     * @param tactic The tactic under test
     *
     * @throw invalid_argument if does not contain an tactic
     */
    void setTactic(std::shared_ptr<Tactic> tactic);

    /**
     * Sets the robot id to run tactic on
     *
     * @param robot_id The robot id to run tactic on
     */
    void setRobotId(RobotId robot_id);

    /**
     * Sets the motion constraints to test
     *
     * @param motion_constraints The motion constraints for the tactic
     */
    void setMotionConstraints(const std::set<MotionConstraint>& motion_constraints);

   private:
    void updatePrimitives(World world,
                          std::shared_ptr<Simulator> simulator_to_update) override;
    std::optional<PlayInfo> getPlayInfo() override;
    AIDrawFunction getDrawFunctions() override;

    std::shared_ptr<Tactic> tactic;
    std::optional<RobotId> robot_id;
    // Motion constraints to set for intent
    std::set<MotionConstraint> motion_constraints;
    // The Navigator
    std::shared_ptr<Navigator> navigator;
};
