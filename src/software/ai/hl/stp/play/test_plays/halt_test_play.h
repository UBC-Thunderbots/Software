#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/strategy/strategy.h"

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
    HaltTestPlay(const TbotsProto::AiConfig &config, std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

   private:
    TbotsProto::AiConfig ai_config;
};
