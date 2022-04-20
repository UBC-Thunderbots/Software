#pragma once

#include <gtest/gtest.h>

#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/navigator/navigator.h"
#include "software/simulated_tests/simulated_er_force_sim_test_fixture.h"

/**
 * This is a test fixture designed to make it easy to write integration tests for Tactics.
 * It provides an interface to set up a single friendly and enemy robot each running a
 * single tactic, and then validate how the world changes over time during simulation
 *
 * Since users can't update the Tactic in sync with simulation ticks, the
 * SimulatedErForceSimTacticTestFixture will update the robot of the tactic
 */
class SimulatedErForceSimTacticTestFixture : public SimulatedErForceSimTestFixture
{
   public:
    explicit SimulatedErForceSimTacticTestFixture();

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
     * Sets the friendly robot ID if the test only needs
     * friendly robots.
     *
     * @param friendly_robot_id The friendly robot ID
     *
     */
    void setFriendlyRobotId(RobotId friendly_robot_id);

    /**
     * Sets the friendly and enemy robot IDs if the test requires both robots
     *
     * @param friendly_robot_id The friendly robot ID
     * @param enemy_robot_id The enemy robot ID
     */
    void setBothRobotId(RobotId friendly_robot_id, RobotId enemy_robot_id);

    /**
     * Sets the motion constraints for the friendly and enemy (if applicable) teams
     *
     * @param friendly_motion_constraints The friendly motion constraint
     * @param enemy_motion_constraints The enemy motion constraint
     */
    void setMotionConstraints(
        const std::set<TbotsProto::MotionConstraint>& friendly_motion_constraints,
        const std::set<TbotsProto::MotionConstraint>& enemy_motion_constraints = {});

   private:
    void updatePrimitives(const World& friendly_world, const World& enemy_world,
                          std::shared_ptr<ErForceSimulator> simulator_to_update) override;

    /**
     * Updates primitives on the given simulator given these inputs
     *
     * @param world the world
     * @param simulator_to_update The simulator to update
     * @param navigator the navigator to use
     * @param robot_id the robot id of the robot to control
     * @param tactic The tactic to run on the robot
     * @param motion_constraints The motion constraints
     * @param config the ThunderbotsConfig
     *
     * @return the tick duration in ms
     */
    static double updatePrimitives(
        const World& world, std::shared_ptr<ErForceSimulator> simulator_to_update,
        std::shared_ptr<Navigator> navigator, RobotId robot_id,
        std::shared_ptr<Tactic> tactic,
        const std::set<TbotsProto::MotionConstraint>& motion_constraints,
        std::shared_ptr<const ThunderbotsConfig> config);

    std::optional<TbotsProto::PlayInfo> getPlayInfo() override;
    AIDrawFunction getDrawFunctions() override;

    std::shared_ptr<Tactic> friendly_tactic;
    std::shared_ptr<Tactic> enemy_tactic;
    std::optional<RobotId> friendly_robot_id_opt;
    std::optional<RobotId> enemy_robot_id_opt;

    // Motion constraints to set for intent
    std::set<TbotsProto::MotionConstraint> friendly_motion_constraints;
    std::set<TbotsProto::MotionConstraint> enemy_motion_constraints;
    // The Navigator
    std::shared_ptr<Navigator> friendly_navigator;
    std::shared_ptr<Navigator> enemy_navigator;
};
