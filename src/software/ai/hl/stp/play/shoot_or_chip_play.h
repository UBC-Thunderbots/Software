#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * The Defense Play tries to grab the ball from the enemy that has it, and all other
 * robots shadow the enemy robots in order of how threatening they are.
 */
class ShootOrChipPlay : public Play
{
   public:
    ShootOrChipPlay(std::shared_ptr<const AiConfig> config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
