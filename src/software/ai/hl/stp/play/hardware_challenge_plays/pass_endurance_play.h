#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play for the pass endurace hardware challenge
 * https://robocup-ssl.github.io/ssl-hardware-challenge-rules/rules.html
 */
class PassEndurancePlay : public Play
{
   public:
    PassEndurancePlay(std::shared_ptr<const AiConfig> config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

   private:
    // 3 robots for this hardware challenge
    const unsigned int NUM_ROBOTS = 3;
};
