#include "software/ai/hl/stp/tactic/penalty_setup_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/tactic/tactic_visitor.h"

PenaltySetupTactic::PenaltySetupTactic(bool loop_forever) : Tactic(loop_forever)
{
    addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    addWhitelistedAvoidArea(AvoidArea::ENEMY_DEFENSE_AREA);
    addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
}

std::string PenaltySetupTactic::getName() const
{
    return "Shooter Setup Tactic";
}

void PenaltySetupTactic::updateControlParams(Point destination, Angle final_orientation,
                                             double final_speed)
{
    // Update the control parameters stored by this Tactic
    this->destination       = destination;
    this->final_orientation = final_orientation;
    this->final_speed       = final_speed;
}

double PenaltySetupTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer robots closer to the destination
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - destination).len() / world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void PenaltySetupTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    MoveAction move_action = MoveAction(0, Angle(), false);
    do
    {
        yield(move_action.updateStateAndGetNextIntent(
            *robot, destination, final_orientation, final_speed, DribblerEnable::OFF,
            MoveType::NORMAL, AutokickType::NONE));
    } while (!move_action.done());
}

void PenaltySetupTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
