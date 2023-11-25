#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_tactic.h"

PenaltyKickTactic::PenaltyKickTactic(TbotsProto::AiConfig ai_config)
    : Tactic({RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick}),
      fsm_map(),
      ai_config(ai_config)
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<PenaltyKickFSM>>(
            DribbleFSM(ai_config.dribble_tactic_config()), PenaltyKickFSM(),
            GetBehindBallFSM());
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
            DribbleFSM(ai_config.dribble_tactic_config()), PenaltyKickFSM(),
            GetBehindBallFSM(), KickFSM());
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(PenaltyKickFSM::Update({}, tactic_update));
}
