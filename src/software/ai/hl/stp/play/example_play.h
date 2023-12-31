#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * An example Play that moves the robots in a circle around the ball
 */
class ExamplePlay : public Play
{
   public:
    explicit ExamplePlay(const TbotsProto::AiConfig& config,
            std::shared_ptr<Strategy> strategy = std::make_shared<Strategy>());

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
