#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/convex_angle.h"
#include "software/logger/logger.h"

ReceiverTactic::ReceiverTactic(std::shared_ptr<Strategy> strategy)
    : SupportTactic({RobotCapability::Move}),
      strategy_(strategy),
      fsm_map(),
      control_params()
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<ReceiverFSM>>(ReceiverFSM(strategy_));
    }
}

void ReceiverTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

void ReceiverTactic::updateReceivingPosition(std::optional<Point> receiving_position)
{
    updateControlParams(receiving_position);
}

void ReceiverTactic::updateControlParams(std::optional<Point> receiving_position,
                                         bool enable_one_touch_shot)
{
    // Update the control parameters stored by this Tactic
    control_params.receiving_position    = receiving_position;
    control_params.enable_one_touch_shot = enable_one_touch_shot;
}

void ReceiverTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] =
            std::make_unique<FSM<ReceiverFSM>>(ReceiverFSM(strategy_));
    }

    fsm_map.at(tactic_update.robot.id())
        ->process_event(ReceiverFSM::Update(control_params, tactic_update));
}
