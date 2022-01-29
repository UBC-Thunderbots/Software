#pragma once

#include <memory>
#include <random>
#include <vector>

#include "proto/play_info_msg.pb.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play_selection_fsm.h"
#include "software/ai/intent/intent.h"

/**
 * The STP module is an implementation of the high-level logic Abstract class, that
 * uses the STP (Skills, Tactics, Plays) framework for its decision making.
 *
 * See play.h and tactic.h for more specific documentation on
 * each component.
 *
 * Explaining ControlParams
 * - ControlParams are any parameters that are related to gameplay logic.
 *   Some examples are:
 *   - The EnemyThreat datastructure
 *   - A Pass datastructure
 *   - A position on the field
 *   - How far to chip the ball
 *   These parameters can NOT be automatically updated because they are specific
 *   to the Play or Tactic that is providing them. Therefore, Plays update the
 *   ControlParams of Tactics. These are the parameters that actually change their
 *   behaviour.
 * - NOTE: In the high-level overview below, the ControlParams
 *   for the Tactics are updated before the Tactics are run, so all
 *   necessary information is provided before gameplay decisions are made.
 *
 * STP's main job is to decide what each robot should be doing at any point
 * in time (ie. what Intent each robot should be running). When STP is given
 * a new World and asked for Intents, it goes through the following steps:
 *
 * 1. Decide what Play should be running at this time. If the play is done then
 *    it is restarted.
 *
 * 2. Run the current Play to get the Tactics should be run at this time.
 *    This will update the ControlParams for the tactics.
 *    NOTE: At this time no particular robot is assigned to run any particular tactic.
 *    We just know WHAT tactics should be run, not what robot is running them.
 *
 * 3. Assign robots to tactics. Tactics assign a "cost" to each robot based on internal
 *    criteria, and then we attempt to assign robots in such a way that minimizes the
 *    global cost.
 *
 * 4. Run each tactic to get the Intent that each robot should run.
 *
 * 5. Return this list of Intents
 */
class STP
{
   public:
    STP() = delete;
    /**
     * Creates a new High-Level logic module that uses the STP framework for
     * decision-making.
     *
     * @param ai_config The Ai configuration
     */
    explicit STP(std::shared_ptr<const AiConfig> ai_config);

    /**
     * Given the state of the world, returns the Intent that each available Robot should
     * be running.
     *
     * @param world The current state of the world
     *
     * @return A vector of unique pointers to the Intents our friendly robots should be
     * running
     */
    std::vector<std::unique_ptr<Intent>> getIntents(const World &world);

    /**
     * Returns information about the currently running plays and tactics, including the
     * name of the play, and which robots are running which tactics
     *
     * @return information about the currently running plays and tactics
     */
    TbotsProto::PlayInfo getPlayInfo();

    /**
     * Overrides the play constructor so whenever STP creates a new play it calls
     * constructor
     *
     * @param constructor the override constructor
     */
    void overridePlayConstructor(std::optional<PlayConstructor> constructor);

    /**
     * Given a vector of vector of tactics and the current World, assigns robots
     * from the friendly team to each tactic
     *
     * Some tactics may not be assigned a robot, depending on if there is a robot
     * capable of performing that tactic
     *
     * This will clear all assigned robots from all tactics
     *
     * The outer vector ranks the inner vector of tactics by priority. Tactics in
     * lower indexes of the outer vector will be assigned first. For example:
     *
     * {
     *      {crease_defender_1, crease_defender_2},
     *      {move_tactic},
     * }
     *
     * The cost of assigning both the crease_defender tactics will be minimized across
     * all robots first, followed by the move_tactic.
     *
     * The order of the given tactics in the inner vector also determines their priority,
     * with the tactics at the beginning of the vector being a higher priority than those
     * at the end. The priority determines which tactics will NOT be assigned if there are
     * not enough robots on the field to assign them all. For example, if a Play returned
     * 4 Tactics in total but there were only 3 robots on the field at the time, only the
     * first 3 Tactics in the vectors would be assigned to robots and run. (In the example
     * above, only the goalie and crease_defenders would be assigned)
     *
     * @param tactics The vector of vector of tactics that should be assigned a robot.
     * Note that this function modifies tactics to make the correct assignments, because
     * we need to modify the individual tactics _and_ possibly add/remove tactics
     * @param world The state of the world, which contains the friendly Robots that will
     * be assigned to each tactic
     * @param automatically_assign_goalie whether or not to automatically assign a goalie
     * tactic
     *
     * @return map from assigned tactics to robot
     */
    std::map<std::shared_ptr<const Tactic>, Robot> assignRobotsToTactics(
        ConstPriorityTacticVector tactics, const World &world,
        bool automatically_assign_goalie);

   private:
    /**
     * Gets the intents the current play wants to run
     *
     * @return The vector of intents that should be run right now to execute the play
     */
    std::vector<std::unique_ptr<Intent>> getIntentsFromCurrentPlay(const World &world);

    void overridePlayName(std::string name);

    // The Play that is currently running
    std::map<std::shared_ptr<const Tactic>, Robot> robot_tactic_assignment;
    std::shared_ptr<const AiConfig> ai_config;
    // Goalie tactic common to all plays
    std::shared_ptr<GoalieTactic> goalie_tactic;
    // Stop tactic common to all plays for robots that don't have tactics assigned
    TacticVector stop_tactics;
    std::unique_ptr<Play> current_play;
    std::unique_ptr<FSM<PlaySelectionFSM>> fsm;
    bool override_play_changed;
    std::optional<PlayConstructor> override_constructor;
};
