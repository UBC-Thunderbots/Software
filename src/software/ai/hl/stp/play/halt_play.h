#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A Play that stops all the robots on the field. This is primarily used to obey the
 * referee "Halt" command, as well as a fallback for when we don't have a play assigned.
 */
class HaltPlay : public Play
{
   public:
    HaltPlay(std::shared_ptr<const AiConfig> config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
