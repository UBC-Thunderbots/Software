#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_tactic.h"

PenaltyKickTactic::PenaltyKickTactic()
    : Tactic(false,
             {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick}),
      fsm{DribbleFSM(), PenaltyKickFSM(), GetBehindBallFSM()}
{
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

bool PenaltyKickTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void PenaltyKickTactic::updateIntent(const TacticUpdate& tactic_update)
{
    fsm.process_event(PenaltyKickFSM::Update({}, tactic_update));
}
