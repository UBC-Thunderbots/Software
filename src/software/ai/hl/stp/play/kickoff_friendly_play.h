#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

#include "software/ai/passing/pass_generator.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"

/**
 * A play that runs when its currently the friendly kick off,
 * only one robot grabs the ball and passes to another robot.
 */
class KickoffFriendlyPlay : public Play
{
   public:
    KickoffFriendlyPlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    
    std::vector<CircleWithColor> getCirclesWithColorToDraw() override;
    
    std::optional<Pass> best_pass;
    bool committed_to_pass = false;
    
    private :
        std::shared_ptr<ReceiverTactic> receiver;
        static double constexpr BETTER_PASS_RATING_THRESHOLD = 0.1;
};
