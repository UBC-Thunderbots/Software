#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"

PassDefenderTactic::PassDefenderTactic()
    : Tactic({RobotCapability::Move, RobotCapability::Kick}),
      fsm_map(),
      control_params(PassDefenderFSM::ControlParams())
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<PassDefenderFSM>>(PassDefenderFSM());
    }
}

void PassDefenderTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void PassDefenderTactic::updateControlParams(const Point &position_to_block_from)
{
    control_params.position_to_block_from = position_to_block_from;
}

void PassDefenderTactic::updatePrimitive(const TacticUpdate &tactic_update,
                                         bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] =
            std::make_unique<FSM<PassDefenderFSM>>(PassDefenderFSM());
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(PassDefenderFSM::Update(control_params, tactic_update));
}
