#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_tactic.h"

PenaltyKickTactic::PenaltyKickTactic(std::shared_ptr<const AiConfig> ai_config)
    : Tactic({RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick}),
      fsm_map()
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<PenaltyKickFSM>>(
            DribbleFSM(ai_config->getDribbleTacticConfig()), PenaltyKickFSM(),
            GetBehindBallFSM());
    }
}

void PenaltyKickTactic::updateControlParams() {}

double PenaltyKickTactic::calculateRobotCost(const Robot& robot, const World& world) const
{
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - world.ball().position()).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void PenaltyKickTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

void PenaltyKickTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<PenaltyKickFSM>>(
            DribbleFSM(), PenaltyKickFSM(), GetBehindBallFSM());
    }
    fsm.process_event(PenaltyKickFSM::Update({}, tactic_update));
}
