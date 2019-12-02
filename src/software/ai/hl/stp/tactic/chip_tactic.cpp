#include "software/ai/hl/stp/tactic/chip_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/chip_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"

ChipTactic::ChipTactic(const Ball &ball, bool loop_forever)
    : Tactic(loop_forever, {RobotCapabilities::Capability::Chip}), ball(ball)
{
}

std::string ChipTactic::getName() const
{
    return "Chip Tactic";
}

void ChipTactic::updateWorldParams(const Ball &ball)
{
    // update the world parameters stored by this tactic
    this->ball = ball;
}

void ChipTactic::updateControlParams(Point chip_origin, Point chip_target,
                                     double chip_distance_meters)
{
    // update the control parameters stored by this tactic
    this->chip_origin          = chip_origin;
    this->chip_target          = chip_target;
    this->chip_distance_meters = chip_distance_meters;
}

double ChipTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // the closer the robot is to a ball, the cheaper it is to perform the chip
    double cost = (robot.position() - world.ball().position()).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void ChipTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto chip_action = std::make_shared<ChipAction>();
    do
    {
        chip_action->updateWorldParams(ball);
        chip_action->updateControlParams(*robot, chip_origin, chip_target,
                                         chip_distance_meters);
        yield(chip_action);
    } while (!chip_action->done());
}

void ChipTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
