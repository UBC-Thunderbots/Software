#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A test Play that halts 3 robots.
 *
 * The Play is never done
 *
 * This play is applicable when the ball's y coordinate is >= 0
 * This play's invariant holds while the ball is within the field
 */
class HaltTestPlay : public Play
{
   public:
    HaltTestPlay(std::shared_ptr<const AiConfig> config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

   private:
    std::shared_ptr<const AiConfig> ai_config;
};
