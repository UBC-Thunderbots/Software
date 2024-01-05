#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * An example Play that moves the robots in a circle around the ball
 */
class MoveToDestAndOrientPlay : public Play // TODO (NIMA): Remove
{
   public:
    explicit MoveToDestAndOrientPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
