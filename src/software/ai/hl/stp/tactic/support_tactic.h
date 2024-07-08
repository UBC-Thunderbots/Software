#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"

class SupportTactic : public Tactic
{
   public:
    using Tactic::Tactic;

    /**
     * Updates the SupportTactic's receiving position on the field.
     * 
     * @param receiving_position the point at which to receive a possible pass
     */
    virtual void updateReceivingPosition(std::optional<Point> receiving_position) = 0;
};