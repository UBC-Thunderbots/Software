#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play for scoring with static defenders play hardware challenge
 * https://robocup-ssl.github.io/ssl-hardware-challenge-rules/rules.html
 */
class ScoringWithStaticDefendersPlay : public Play
{
   public:
    ScoringWithStaticDefendersPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

   private:
    // 3 robots for this hardware challenge
    const unsigned int NUM_ROBOTS = 3;
};
