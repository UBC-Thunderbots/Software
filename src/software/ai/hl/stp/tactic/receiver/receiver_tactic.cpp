#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/convex_angle.h"
#include "software/logger/logger.h"

ReceiverTactic::ReceiverTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<ReceiverFSM>({RobotCapability::Move}, ai_config_ptr),
      control_params({ReceiverFSMControlParams{.pass                   = std::nullopt,
                                                 .disable_one_touch_shot = false}})
{
}

void ReceiverTactic::updateControlParams(std::optional<Pass> updated_pass,
                                         bool disable_one_touch_shot)
{
    // Update the control parameters stored by this Tactic
    control_params.pass                   = updated_pass;
    control_params.disable_one_touch_shot = disable_one_touch_shot;
}

void ReceiverTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

void ReceiverTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = fsm_init();
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(ReceiverFSM::Update(control_params, tactic_update));
}
