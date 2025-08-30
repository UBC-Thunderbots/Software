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
    ScoringFromContestedPossessionPlay(
        std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
};
