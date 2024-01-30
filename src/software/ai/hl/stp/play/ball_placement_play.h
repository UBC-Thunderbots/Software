#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A Play that performs ball placement, i.e. placing the ball in a defined location
 * determined by the referee. This is used to obey the referee "Ball Placement Us" command
 */
class BallPlacementPlay : public Play
{
   public:
    BallPlacementPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const WorldPtr &world_ptr) override;
};
