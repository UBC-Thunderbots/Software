#ifndef AI_HL_STP_STPHL_H_
#define AI_HL_STP_STPHL_H_

#include "ai/hl/hl.h"
#include "ai/intent.h"
#include "ai/world/world.h"

/**
 * The STPHL module is an implementation of the high-level logic Abstract class, that
 * uses the STP (Skills, Tactics, Plays) framework for its decision making.
 */
class STPHL : public HL
{
   public:
    /**
     * Creates a new High-Level logic module that uses the STP framework for
     * decision-making
     */
    explicit STPHL();

    /**
    * Given the state of the world, returns the intent that each available Robot should be
    * running.
    *
    * @param world The current state of the world
    * @return A vector of pairs, where each pair contains a robot id and the Intent that
    * robot should run
    */
    std::vector<std::pair<unsigned int, Intent>> getIntentAssignment(
        const World &world) override;
};

#endif  // AI_HL_STP_STPHL_H_
