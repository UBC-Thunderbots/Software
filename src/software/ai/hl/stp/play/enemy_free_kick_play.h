#pragma once

#include <memory>
#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/strategy/strategy.h"

/**
 * Play for defending against enemy free kicks
 */
class EnemyFreekickPlay : public Play
{
   public:
    EnemyFreekickPlay(const TbotsProto::AiConfig &config, std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
