#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/strategy/strategy.h"

/**
 * A Play that performs ball placement, i.e. placing the ball in a defined location
 * determined by the referee. This is used to obey the referee "Ball Placement Us" command
 */
class BallPlacementPlay : public Play
{
   public:
    BallPlacementPlay(std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
