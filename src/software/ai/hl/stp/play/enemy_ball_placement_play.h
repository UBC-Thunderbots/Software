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
    void defendingBallPlacement(
        TacticCoroutine::push_type &yield,
        const World &world,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 3> crease_defenders,
        std::array<std::shared_ptr<MoveTactic>, 2> move_tactics,
        Point placement_point);

    void attackingBallPlacement(
        TacticCoroutine::push_type &yield,
        const World &world,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 3> crease_defenders,
        std::array<std::shared_ptr<MoveTactic>, 2> move_tactics,
        Point placement_point);
};

