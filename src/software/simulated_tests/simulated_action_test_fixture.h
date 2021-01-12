#pragma once

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/action.h"
#include "software/ai/navigator/navigator.h"
#include "software/simulated_tests/simulated_test_fixture.h"

/**
 * This is a test fixture designed to make it easy to write integration tests for Actions.
 * It provides an easy interface to set up a single robot running a single action , and
 * then validate how the world changes over time during simulation
 *
 * Since users can't update the Action in sync with simulation ticks, the
 * SimulatedActionTestFixture will update the robot of the action
 */
class SimulatedActionTestFixture : public SimulatedTestFixture
{
   public:
    explicit SimulatedActionTestFixture();

   protected:
    void SetUp() override;

    /**
     * Sets the action to test
     *
     * @param action The action under test
     *
     * @throw invalid_argument if does not contain an action
     */
    void setAction(std::shared_ptr<Action> action);

    /**
     * Sets the motion constraints to test
     *
     * @param motion_constraints The motion constraints for the action
     */
    void setMotionConstraints(const std::set<MotionConstraint>& motion_constraints);

   private:
    void updatePrimitives(const World& world,
                          std::shared_ptr<Simulator> simulator_to_update) override;
    std::optional<PlayInfo> getPlayInfo() override;
    AIDrawFunction getDrawFunctions() override;

    std::shared_ptr<Action> action;
    // Motion constraints to set for intent
    std::set<MotionConstraint> motion_constraints;
    // The Navigator
    std::shared_ptr<Navigator> navigator;
};
