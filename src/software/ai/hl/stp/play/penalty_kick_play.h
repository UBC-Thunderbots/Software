#pragma once

#include "software/ai/hl/stp/play/play.h"
#include "shared/parameter_v2/cpp_dynamic_parameters.h"

/**
 * Play for shooting penalty kicks
 */
class PenaltyKickPlay : public Play
{
   public:
    PenaltyKickPlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
