#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A play that runs when its currently the friendly kick off,
 * only one robot grabs the ball and passes to another robot.
 */
class KickoffFriendlyPlay : public Play
{
   public:
    KickoffFriendlyPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
