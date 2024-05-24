#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/convex_angle.h"
#include "software/logger/logger.h"

ReceiverTactic::ReceiverTactic(std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Move}),
      strategy_(strategy),
      fsm_map(),
      control_params({ReceiverFSM::ControlParams{.receiver_point = std::nullopt,
                                                 .disable_one_touch_shot = false}})
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

std::map<RobotId, std::shared_ptr<Primitive>> ReceiverTactic::get(
    const WorldPtr& world_ptr)
{
    std::optional<Pass> committed_pass = strategy_->getNextCommittedPass();
    if (committed_pass)
    {
        control_params.receiver_point = committed_pass->receiverPoint();
    }
    else 
    {
        control_params.receiver_point = strategy_->getNextBestReceivingPosition();
    }

    return Tactic::get(world_ptr);
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
