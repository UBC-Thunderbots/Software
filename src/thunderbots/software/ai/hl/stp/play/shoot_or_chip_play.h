#pragma once

#include "ai/hl/stp/play/play.h"

/**
 * The Defense Play tries to grab the ball from the enemy that has it, and all other
 * robots shadow the enemy robots in order of how threatening they are.
 */
class ShootOrChipPlay : public Play
{
   public:
    static const std::string name;

    ShootOrChipPlay();

    std::string getName() const override;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield) override;

private:

    // The minimum open net percentage we will try to shoot at
    const double MIN_NET_PERCENT_OPEN_FOR_SHOT;

    const double CHIP_TARGET_FIELD_INSET;

};
