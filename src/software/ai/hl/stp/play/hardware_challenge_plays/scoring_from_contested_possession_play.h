#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play for the scoring from contested possession hardware challenge
 * https://robocup-ssl.github.io/ssl-hardware-challenge-rules/rules.html
 */
class ScoringFromContestedPossessionPlay : public Play
{
   public:
    ScoringFromContestedPossessionPlay(const TbotsProto::AiConfig& config,
            std::shared_ptr<Strategy> strategy = std::make_shared<Strategy>());

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
