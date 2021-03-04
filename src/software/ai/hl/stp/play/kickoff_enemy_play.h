#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A play that runs when its currently the enemies kick off,
 * prioritizes defending the net and shadowing the robot
 * that is nearest to the ball. Any remaining bots will block
 * some odd angles to the net.
 */
class KickoffEnemyPlay : public Play
{
   public:
    KickoffEnemyPlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
