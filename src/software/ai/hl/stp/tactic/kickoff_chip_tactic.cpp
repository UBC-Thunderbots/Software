#include "software/ai/hl/stp/tactic/kickoff_chip_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/chip_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"

KickoffChipTactic::KickoffChipTactic(const Ball &ball, bool loop_forever)
    : Tactic(loop_forever, {RobotCapabilityFlags::Chip}), ball(ball)
{
    // the chipper is allowed to go into the centre circle and touch the ball
    addWhitelistedAvoidArea(AvoidArea::CENTER_CIRCLE);
    addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
}

std::string KickoffChipTactic::getName() const
{
    return "Kickoff Chip Tactic";
}

void KickoffChipTactic::updateWorldParams(const Ball &ball)
{
    // update the world parameters stored by this tactic
    this->ball = ball;
}

void KickoffChipTactic::updateControlParams(Point chip_origin, Point chip_target,
                                            double chip_distance_meters)
{
    // update the control parameters stored by this tactic
    this->chip_origin          = chip_origin;
    this->chip_target          = chip_target;
    this->chip_distance_meters = chip_distance_meters;
}

double KickoffChipTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // the closer the robot is to a ball, the cheaper it is to perform the chip
    double cost =
        (robot.position() - world.ball().position()).len() / world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void KickoffChipTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    ChipAction chip_action = ChipAction();
    do
    {
        yield(chip_action.updateStateAndGetNextIntent(*robot, ball, chip_origin,
                                                      chip_target, chip_distance_meters));
    } while (!chip_action.done());
}

void KickoffChipTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
