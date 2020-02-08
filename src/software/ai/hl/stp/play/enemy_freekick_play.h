#pragma once

#include "software/ai/hl/stp/play/play.h"
#include "software/parameter/dynamic_parameters.h"

/**
 * Play for defending against enemy free kicks
 */
class EnemyFreekickPlay : public Play
{
   public:
    static const std::string name;

    EnemyFreekickPlay(std::shared_ptr<EnemyCapabilityConfig> enemy_capability_config);

    std::string getName() const override;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield) override;

   private:
    std::shared_ptr<EnemyCapabilityConfig> enemy_capability_config;
};
