#include "ai/hl/stp/tactic/chip_tactic.h"

#include <algorithm>

#include "ai/hl/stp/action/chip_action.h"

ChipTactic::ChipTactic(const Ball &ball, bool loop_forever)
    : ball(ball), Tactic(loop_forever, {RobotCapabilityFlags::Chip})
{
}

std::string ChipTactic::getName() const
{
    return "Chip Tactic";
}

void ChipTactic::updateParams(const Ball &ball, Point chip_origin, Point chip_target,
                              double chip_distance_meters)
{
    // update the parameters stored by this tactic
    this->ball                 = ball;
    this->chip_origin          = chip_origin;
    this->chip_target          = chip_target;
    this->chip_distance_meters = chip_distance_meters;
}

double ChipTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // the closer the robot is to a ball, the cheaper it is to perform the chip
    double cost =
        (robot.position() - world.ball().position()).len() / world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

void ChipTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    ChipAction chip_action = ChipAction();
    do
    {
        yield(chip_action.updateStateAndGetNextIntent(*robot, ball, chip_origin,
                                                      chip_target, chip_distance_meters));
    } while (!chip_action.done());
}
