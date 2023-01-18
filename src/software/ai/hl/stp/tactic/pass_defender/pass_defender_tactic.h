#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/point.h"

/**
 * A pass defender moves to a location on the field to block and intercept
 * a potential pass between enemy robots.
 *  
 */
class PassDefenderTactic : public Tactic
{
   public:
    explicit PassDefenderTactic();

    /**
     * Update control params for this tactic
     *
     * @param position_to_block_from The location on the field to block enemy passes from
     */
    void updateControlParams(const Point& position_to_block_from);

    void accept(TacticVisitor& visitor) const override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    std::map<RobotId, std::unique_ptr<FSM<PassDefenderFSM>>> fsm_map;

    PassDefenderFSM::ControlParams control_params;
};
