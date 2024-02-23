#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"

/**
 * A play to set up robots during enemy ball placement
 */
class EnemyBallPlacementPlay : public Play
{
   public:
    EnemyBallPlacementPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
