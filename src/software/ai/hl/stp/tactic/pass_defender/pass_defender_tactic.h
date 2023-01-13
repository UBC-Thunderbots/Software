#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/point.h"

class PassDefenderTactic : public Tactic
{
   public:
    explicit PassDefenderTactic();

    /**
     * Update control params for this tactic
     *
     * @param position_to_block The location to block passes from
     */
    void updateControlParams(const Point& position_to_block);

    void accept(TacticVisitor& visitor) const override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    std::map<RobotId, std::unique_ptr<FSM<PassDefenderFSM>>> fsm_map;

    PassDefenderFSM::ControlParams control_params;
};
