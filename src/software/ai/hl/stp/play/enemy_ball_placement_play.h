#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"

/**
 * A play to set up robots during enemy ball placement
 */
class EnemyBallPlacementPlay : public Play
{
   public:
    EnemyBallPlacementPlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

   private:
    void inAttackingZone(
        TacticCoroutine::push_type &yield,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 3> crease_defender_tactics,
        const World &world);

    void inDefendingZone(
        TacticCoroutine::push_type &yield,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 4> crease_defender_tactics,
        const World &world);
};
