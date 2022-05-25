#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play for shooting penalty kicks
 */
class PenaltyKickPlay : public Play
{
   public:
    PenaltyKickPlay(std::shared_ptr<const AiConfig> config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
