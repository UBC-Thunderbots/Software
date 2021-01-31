#include "software/ai/hl/stp/tactic/chip_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/chip_action.h"

ChipTactic::ChipTactic(const Ball &ball, bool loop_forever)
    : Tactic(loop_forever, {RobotCapability::Chip, RobotCapability::Move}), ball(ball)
{
}


void ChipTactic::updateWorldParams(const World &world)
{
    this->ball = world.ball();
}


void ChipTactic::updateControlParams(Point chip_origin, Point chip_target)
{
    // update the control parameters stored by this tactic
    this->chip_origin = chip_origin;
    this->chip_target = chip_target;
}

double ChipTactic::calculateRobotCost(const Robot &robot, const World &world) const
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
        chip_action->updateControlParams(*robot_, chip_origin, chip_target);
        yield(chip_action);
    } while (!chip_action->done());
}

void ChipTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

Ball ChipTactic::getBall() const
{
    return this->ball;
}
