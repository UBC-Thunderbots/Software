#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play for defending penalty kicks
 */
class PenaltyKickEnemyPlay : public Play
{
   public:
    PenaltyKickEnemyPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
