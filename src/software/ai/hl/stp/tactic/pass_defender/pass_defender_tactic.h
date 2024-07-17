#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/point.h"

/**
 * A pass defender moves to a location on the field to block a potential
 * pass between enemy robots + intercepts and chips the ball away when
 * an active enemy pass is directed towards the defender.
 *
 */
class PassDefenderTactic : public Tactic
{
   public:
    /**
     * Creates a new PassDefenderTactic
     *
     * @param strategy the Strategy shared by all of AI
     */
    explicit PassDefenderTactic(std::shared_ptr<Strategy> strategy);

    /**
     * Update control params for this tactic
     *
     * @param position_to_block_from The location on the field to block enemy passes from
     * @param ball_steal_mode The pass defender's aggressiveness towards the ball
     */
    void updateControlParams(const Point& position_to_block_from,
                             TbotsProto::BallStealMode ball_steal_mode);

    void accept(TacticVisitor& visitor) const override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    std::shared_ptr<Strategy> strategy;
    std::map<RobotId, std::unique_ptr<FSM<PassDefenderFSM>>> fsm_map;
    PassDefenderFSM::ControlParams control_params;
};
