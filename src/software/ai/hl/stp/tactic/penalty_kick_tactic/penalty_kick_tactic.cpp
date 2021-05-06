/**
 * Implementation of the PenaltyKickTactic
 */
#include "software/ai/hl/stp/tactic/penalty_kick_tactic/penalty_kick_tactic.h"

PenaltyKickTactic::PenaltyKickTactic(const Ball& ball, bool loop_forever,
                                     const std::optional<Robot>& enemy_goalie)
    : Tactic(loop_forever,
             {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick}),
      ball(ball),
      enemy_goalie(enemy_goalie),
      control_params({.enemy_goalie = enemy_goalie})
{
}

void PenaltyKickTactic::updateWorldParams(const World& world) {}

void PenaltyKickTactic::updateControlParams()
{
    control_params.enemy_goalie = enemy_goalie;
}

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

Ball PenaltyKickTactic::getBall() const
{
    return this->ball;
}

bool PenaltyKickTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void PenaltyKickTactic::updateIntent(const TacticUpdate& tactic_update)
{
    fsm.process_event(PenaltyKickTacticFSM::Update(control_params, tactic_update));
}
