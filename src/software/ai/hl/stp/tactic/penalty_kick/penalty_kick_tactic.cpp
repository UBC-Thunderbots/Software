#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_tactic.h"

PenaltyKickTactic::PenaltyKickTactic()
    : Tactic(false,
             {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick}),
      fsm(DribbleFSM(std::make_shared<Point>()),
          PenaltyKickFSM(std::nullopt, Point(), Angle()))
{
}

void PenaltyKickTactic::updateWorldParams(const World& world) {}

void PenaltyKickTactic::updateControlParams() {}

double PenaltyKickTactic::calculateRobotCost(const Robot& robot, const World& world) const
{
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - world.ball().position()).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void PenaltyKickTactic::calculateNextAction(ActionCoroutine::push_type& yield)
{
    auto stop_action = std::make_shared<StopAction>(false);

    do
    {
        stop_action->updateControlParams(*robot_, true);
        yield(stop_action);
    } while (!stop_action->done());
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
