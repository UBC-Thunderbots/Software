#include "ai/hl/stp/stp_hl.h"
#include "ai/intent/move_intent.h"

STP_HL::STP_HL()
{
}

std::vector<std::unique_ptr<Intent>> STP_HL::getIntentAssignment(const World &world) const
{
    // TODO: Implement this
    // https://github.com/UBC-Thunderbots/Software/issues/24
    return std::vector<std::unique_ptr<Intent>>();
}
