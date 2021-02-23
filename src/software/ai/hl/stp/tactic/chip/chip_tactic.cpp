#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"

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

void ChipTactic::updateControlParams(const Point &chip_origin,
                                     const Angle &chip_direction,
                                     double chip_distance_meters)
{
    control_params.chip_origin          = chip_origin;
    control_params.chip_direction       = chip_direction;
    control_params.chip_distance_meters = chip_distance_meters;
}

void ChipTactic::updateControlParams(const Point &chip_origin, const Point &chip_target)
{
    updateControlParams(chip_origin, (chip_target - chip_origin).orientation(),
                        (chip_target - chip_origin).length());
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
        chip_action->updateControlParams(*robot_, control_params.chip_origin,
                                         control_params.chip_direction,
                                         control_params.chip_distance_meters);
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

bool ChipTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void ChipTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(ChipFSM::Update(control_params, tactic_update));
}
