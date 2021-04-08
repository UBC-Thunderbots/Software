#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play for defending against enemy free kicks
 */
class EnemyFreekickPlay : public Play
{
   public:
    EnemyFreekickPlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
