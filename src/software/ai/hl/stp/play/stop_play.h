#pragma once

#include "software/ai/hl/stp/play/play.h"
#include "software/parameter/dynamic_parameters.h"

/**
 * This Play moves our robots in a formation while keeping them at least 0.5m from the
 * ball. Additionally, the robots are limited to moving no more than 1.5m/s. This Play is
 * used during the referee "Stop" command.
 */
class StopPlay : public Play
{
   public:
    StopPlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

   private:
    std::shared_ptr<const PlayConfig> play_config;
};
