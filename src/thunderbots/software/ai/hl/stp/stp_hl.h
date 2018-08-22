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
    std::vector<std::pair<Robot, std::unique_ptr<Tactic>>> assignTacticsToRobots(
        const World& world, const std::vector<std::unique_ptr<Tactic>>& tactics) const;
    std::shared_ptr<Play> calculateNewPlay(const World& world) const;

    std::shared_ptr<Play> current_play;
};
