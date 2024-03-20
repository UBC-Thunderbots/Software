#include "software/ai/hl/stp/tactic/offense_support_tactics/receiver/receiver_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/convex_angle.h"
#include "software/logger/logger.h"

ReceiverTactic::ReceiverTactic(std::shared_ptr<Strategy> strategy)
    : OffenseSupportTactic({RobotCapability::Move}, strategy),
      fsm_map(),
      control_params({ReceiverFSM::ControlParams{.pass                   = std::nullopt,
                                                 .disable_one_touch_shot = false}})
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<ReceiverFSM>>(ReceiverFSM());
    }
}

OffenseSupportType ReceiverTactic::getOffenseSupportType() const
{
    return OffenseSupportType::PASS_RECEIVER;
}

void ReceiverTactic::commit()
{
    OffenseSupportTactic::commit();

    PassWithRating best_pass = (*strategy_)->getBestUncommittedPass();

    control_params.pass = best_pass.pass;
    control_params.disable_one_touch_shot = true;

    (*strategy_)->commit(best_pass.pass);
}

void ReceiverTactic::updateControlParams(std::optional<Pass> updated_pass,
                                         bool disable_one_touch_shot)
{
    // Update the control parameters stored by this Tactic
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
        fsm_map[tactic_update.robot.id()] =
            std::make_unique<FSM<ReceiverFSM>>(ReceiverFSM());
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(ReceiverFSM::Update(control_params, tactic_update));
}
