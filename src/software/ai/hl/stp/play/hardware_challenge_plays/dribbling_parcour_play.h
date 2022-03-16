#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play for the dribbling parcour hardware challenge
 * https://robocup-ssl.github.io/ssl-hardware-challenge-rules/rules.html
 */
class DribblingParcourPlay : public Play
{
   public:
    DribblingParcourPlay(std::shared_ptr<const AiConfig> config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
