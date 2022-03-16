#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * This Play moves our robots in a formation while keeping them at least 0.5m from the
 * ball. Additionally, the robots are limited to moving no more than 1.5m/s. This Play is
 * used during the referee "Stop" command.
 */
class StopPlay : public Play
{
   public:
    StopPlay(std::shared_ptr<const AiConfig> config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
