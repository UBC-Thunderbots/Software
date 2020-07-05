#pragma once

#include <random>

#include "software/ai/hl/hl.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/intent/intent.h"
#include "software/ai/motion_constraint/motion_constraint_manager.h"
#include "software/parameter/dynamic_parameters.h"

/**
 * The STP module is an implementation of the high-level logic Abstract class, that
 * uses the STP (Skills, Tactics, Plays) framework for its decision making.
 */
class STP : public HL
{
   public:
    /**
     * Creates a new High-Level logic module that uses the STP framework for
     * decision-making.
     *
     * @param default_play_constructor A function that constructs and returns a unique ptr
     * to a Play. This constructor will be used to return a Play if no other Play is
     * applicable during gameplay.
     * @param random_seed The random seed used for STP's internal random number generator.
     * The default value is 0
     */
    explicit STP(std::function<std::unique_ptr<Play>()> default_play_constructor,
                 std::shared_ptr<const AIControlConfig> control_config,
                 long random_seed = 0);

    std::vector<std::unique_ptr<Intent>> getIntents(const World &world) override;

    /**
     * Given a list of tactics and the current World, assigns robots from the friendly
     * team to each tactic
     *
     * Some tactics may not be assigned a robot, depending on if there is a robot
     * capable of performing that tactic
     *
     * This will clear all assigned robots from all tactics
     *
     * The order of the given tactics determines their priority, with the tactics as the
     * beginning of the vector being a higher priority than those at the end. The priority
     * determines which tactics will NOT be assigned if there are not enough robots on the
     * field to assign them all. For example, if a Play returned 6 Tactics but there were
     * only 4 robots on the field at the time, only the first 4 Tactics in the vector
     * would be assigned to robots and run.
     *
     * @param world The state of the world, which contains the friendly Robots that will
     * be assigned to each tactic
     * @param tactics The list of tactics that should be assigned a robot
     */
    void assignRobotsToTactics(const World &world,
                               std::vector<std::shared_ptr<Tactic>> tactics) const;

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

    // A function that constructs a Play that will be used if no other Plays are
    // applicable
    std::function<std::unique_ptr<Play>()> default_play_constructor;
    // The Play that is currently running
    std::unique_ptr<Play> current_play;
    std::optional<std::vector<std::shared_ptr<Tactic>>> current_tactics;
    // The random number generator
    std::mt19937 random_number_generator;
    std::shared_ptr<const AIControlConfig> control_config;
    std::string override_play_name;
    std::string previous_override_play_name;
    bool override_play;
    bool previous_override_play;
    GameState current_game_state;
    MotionConstraintManager motion_constraint_manager;
};
