#pragma once

#include "ai/hl/hl.h"
#include "ai/intent/intent.h"

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

    std::vector<std::unique_ptr<Intent>> getIntentAssignment(const World &world) override;
};
