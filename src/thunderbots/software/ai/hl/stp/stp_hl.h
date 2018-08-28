#pragma once

#include "ai/hl/hl.h"
#include "ai/hl/stp/play/play.h"
#include "ai/intent/intent.h"

/**
 * The STPHL module is an implementation of the high-level logic Abstract class, that
 * uses the STP (Skills, Tactics, Plays) framework for its decision making.
 */
class STP_HL : public HL
{
   public:
    /**
     * Creates a new High-Level logic module that uses the STP framework for
     * decision-making
     */
    explicit STP_HL();

    std::vector<std::unique_ptr<Intent>> getIntentAssignment(const World& world) override;

   private:
    /**
     * Given a list of tactics, assigns each available friendly Robot to the tactic it
     * should run, and returns a pairing of Robots to Tactics
     * @param world The state of the world, which contains the friendly Robots that will
     * be mapped to a Tactic
     * @param tactics The list of tactics that should be run (and paired with a Robot)
     * @return A list of pairs, where each pair contains a Robot and the Tactic that the
     * Robot will run
     */
    std::vector<std::pair<Robot, std::unique_ptr<Tactic>>> assignTacticsToRobots(
        const World& world, const std::vector<std::unique_ptr<Tactic>>& tactics) const;

    /**
     * Given the state of the world, returns the Play that should be run at this time.
     * @param world The world object containing the current state of the world
     * @return A shared pointer to the Play that should be run by the AI
     */
    std::shared_ptr<Play> calculateNewPlay(const World& world) const;

    // The Play that is currently running
    std::shared_ptr<Play> current_play;
};
