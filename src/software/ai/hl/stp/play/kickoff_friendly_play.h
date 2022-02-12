#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A play that runs when its currently the friendly kick off,
 * only one robot grabs the ball and passes to another robot.
 */
class KickoffFriendlyPlay : public Play
{
   public:
    KickoffFriendlyPlay(std::shared_ptr<const AiConfig> config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
