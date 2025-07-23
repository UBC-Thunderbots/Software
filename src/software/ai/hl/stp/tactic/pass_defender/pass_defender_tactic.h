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
class PassDefenderTactic : public Tactic<PassDefenderFSM>
{
   public:
    /**
     * Creates a new PassDefenderTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit PassDefenderTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Update control params for this tactic
     *
     * @param position_to_block_from The location on the field to block enemy passes from
     * @param ball_steal_mode The pass defender's aggressiveness towards the ball
     */
    void updateControlParams(const Point& position_to_block_from,
                             TbotsProto::BallStealMode ball_steal_mode);

    void accept(TacticVisitor& visitor) const override;

   private:
    std::unique_ptr<FSM<PassDefenderFSM>> fsm_init() override;
};
