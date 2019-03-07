#pragma once

#include "ai/hl/hl.h"
#include "ai/hl/stp/play/play.h"
#include "ai/intent/intent.h"
#include <random>

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
     * @param random_seed The random seed used for STP's internal random number generator.
     * Defaults to the number of nanoseconds since the Linux epoch
     */
    explicit STP(long random_seed=std::chrono::system_clock::now().time_since_epoch().count());

    std::vector<std::unique_ptr<Intent>> getIntentAssignment(const World& world) override;

    /**
     * Given a list of tactics, assigns each available friendly Robot to the tactic it
     * should run, and returns a pairing of Robots to Tactics
     *
     * The priority determines what Tactics will not
     * be assigned if there are not enough robots on the field to assign them all. For
     * example, if a Play returned 6 Tactics but there were only 4 robots on the field
     * at the time, only the first 4 Tactics in the vector would be assigned to robots
     * and run.
     *
     * @param world The state of the world, which contains the friendly Robots that will
     * be mapped to a Tactic
     * @param tactics The list of tactics that should be run (and paired with a Robot)
     * @return A list of pairs, where each pair contains a Robot and the Tactic that the
     * Robot will run
     */
    std::vector<std::shared_ptr<Tactic>> assignRobotsToTactics(
        const World& world, std::vector<std::shared_ptr<Tactic>> tactics) const;

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
    std::unique_ptr<Play> calculateNewPlay(const World& world);

    /**
     * Returns the name of the current play, if the current play is assigned. Otherwise
     * returns std::nullopt
     *
     * @return the name of the current play, if the current play is assigned. Otherwise
     * returns std::nullopt
     */
    std::optional<std::string> getCurrentPlayName() const;

private:
    // The Play that is currently running
    std::unique_ptr<Play> current_play;
    // The random number generator
    std::mt19937 random_number_generator;
    // The distribution used to generate numbers
    std::uniform_int_distribution<std::mt19937::result_type> uniform_distribution;
};
