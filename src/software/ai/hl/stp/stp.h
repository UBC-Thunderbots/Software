#pragma once

#include <memory>
#include <random>
#include <vector>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/hl.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/intent/intent.h"

/**
 * The STP module is an implementation of the high-level logic Abstract class, that
 * uses the STP (Skills, Tactics, Plays) framework for its decision making.
 *
 * See play.h, tactic.h, and action.h for more specific documentation on
 * each component.
 *
 * Explaining WorldParams and ControlParams
 * - WorldParams are any parameters that can be derived from the World
 *   class (eg. the ball, field) that are not related to gameplay logic.
 *   In the case of Tactics and Actions, because these parameters are not
 *   related to gameplay logic, we can automatically update them from STP.
 *   This means the Plays that call Tactics, and Tactics that call Actions,
 *   need to provide fewer parameters which overall makes the code a bit
 *   easier to read.
 * - ControlParams are any parameters that are related to gameplay logic.
 *   Some examples are:
 *   - The EnemyThreat datastructure
 *   - A Pass datastructure
 *   - A position on the field
 *   - How far to chip the ball
 *   These parameters can NOT be automatically updated because they are specific
 *   to the Play or Tactic that is providing them. Therefore, Plays update the
 *   ControlParams of Tactics, and Tactics update the ControlParams of Actions.
 *   These are the parameters that actually change their behaviour.
 * - NOTE: In the high-level overview below, the ControlParams and WorldParams
 *   are both updated before something is run (eg. the Control and WorldParams
 *   for the Tactics are both updated before the Tactics are run), so all
 *   necessary information is provided before gameplay decisions are made.
 *
 * HIGH-LEVEL OVERVIEW
 * STP's main job is to decide what each robot should be doing at any point
 * in time (ie. what Intent each robot should be running). When STP is given
 * a new World and asked for Intents, it goes through the following steps:
 *
 * 1. Decide what Play should be running at this time. If the currently running
 *    Play's invariant condition is still valid, that play is chosen to keep running.
 *    If the invariant is no longer true, the play is stopped and a new Play whose
 *    applicable condition is true will be chosen and started.
 *
 *    Altogether it's this constant switching of Plays in reaction to the current
 *    state of the world that makes our AI "play soccer".
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
 * 4. Update the WorldParams for the tactics.
 *
 * 5. Run each tactic to get the Action that each robot should run.
 *    This will update the ControlParams for each Action.
 *
 * 6. Update the "WorldParams" for the Actions
 *
 * 7. Run each Action to get the Intent that each robot should run.
 *
 * 8. Return this list of Intents
 */
class STP : public HL
{
   public:
    STP() = delete;
    /**
     * Creates a new High-Level logic module that uses the STP framework for
     * decision-making.
     *
     * @param default_play_constructor A function that constructs and returns a unique ptr
     * to a Play. This constructor will be used to return a Play if no other Play is
     * applicable during gameplay.
     * @param control_config The Ai Control configuration
     * @param play_config The Play configuration
     * @param random_seed The random seed used for STP's internal random number generator.
     * The default value is 0
     */
    explicit STP(std::function<std::unique_ptr<Play>()> default_play_constructor,
                 std::shared_ptr<const AiControlConfig> control_config,
                 std::shared_ptr<const PlayConfig> play_config, long random_seed);

    std::vector<std::unique_ptr<Intent>> getIntents(const World &world) override;

    /**
     * Given the state of the world, returns a unique_ptr to the Play that should be run
     * at this time. If multiple Plays are applicable and could be run at a given time,
     * one of them is chosen randomly.
     *
     * @param world The world object containing the current state of the world
     * @throws std::runtime error if there are no plays that can be run (ie. no plays
     * are applicable) for the given World state
     * @return A unique pointer to the Play that should be run by the AI
     */
    std::unique_ptr<Play> calculateNewPlay(const World &world);

    /**
     * Returns the name of the current play, if the current play is assigned. Otherwise
     * returns std::nullopt
     *
     * @return the name of the current play, if the current play is assigned. Otherwise
     * returns std::nullopt
     */
    std::optional<std::string> getCurrentPlayName() const;

    /**
     * Returns information about the currently running plays and tactics, including the
     * name of the play, and which robots are running which tactics
     *
     * @return information about the currently running plays and tactics
     */
    PlayInfo getPlayInfo() override;

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
     * Updates the current STP state based on the state of the world
     *
     * @param world
     */
    void updateSTPState(const World &world);

    /**
     * Updates the current game state based on the state of the world
     *
     * @param world
     */
    void updateGameState(const World &world);

    /**
     * Updates the current AI play based on the state of the world
     *
     * @param world
     */
    void updateAIPlay(const World &world);

    /**
     * Gets the intents the current play wants to run
     *
     * @return The vector of intents that should be run right now to execute the play
     */
    std::vector<std::unique_ptr<Intent>> getIntentsFromCurrentPlay(const World &world);

    /**
     * Overrides the AI Play if override is true
     *
     * @return if AI Play was overridden
     */
    bool overrideAIPlayIfApplicable();

    // A function that constructs a Play that will be used if no other Plays are
    // applicable
    std::function<std::unique_ptr<Play>()> default_play_constructor;
    // The Play that is currently running
    std::unique_ptr<Play> current_play;
    std::map<std::shared_ptr<const Tactic>, Robot> robot_tactic_assignment;

    // The random number generator
    std::mt19937 random_number_generator;
    std::shared_ptr<const AiControlConfig> control_config;
    std::shared_ptr<const PlayConfig> play_config;
    std::string override_play_name;
    std::string previous_override_play_name;
    bool override_play;
    bool previous_override_play;
    GameState current_game_state;
    // Goalie tactic common to all plays
    std::shared_ptr<GoalieTactic> goalie_tactic;
    // Stop tactic common to all plays for robots that don't have tactics assigned
    TacticVector stop_tactics;

    static double constexpr NEW_TACTIC_ASSIGNMENT_THRESHOLD = 0.007;
};
