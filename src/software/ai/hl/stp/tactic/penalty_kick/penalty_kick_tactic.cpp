#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_tactic.h"

PenaltyKickTactic::PenaltyKickTactic(std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick}),
      strategy(strategy),
      fsm_map()
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<PenaltyKickFSM>>(
            PenaltyKickFSM(strategy), DribbleSkillFSM(), PivotKickSkillFSM());
    }
}

void PenaltyKickTactic::updateControlParams() {}

void PenaltyKickTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

void PenaltyKickTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<PenaltyKickFSM>>(
            PenaltyKickFSM(strategy), DribbleSkillFSM(), PivotKickSkillFSM());
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(PenaltyKickFSM::Update({}, tactic_update));
}
