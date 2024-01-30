#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A Play that stops all the robots on the field. This is primarily used to obey the
 * referee "Halt" command, as well as a fallback for when we don't have a play assigned.
 */
class HaltPlay : public Play
{
   public:
    HaltPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const WorldPtr &world_ptr) override;
};
