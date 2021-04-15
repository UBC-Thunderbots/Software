#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/point.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"
#include "software/logger/logger.h"

CreaseDefenderTactic::CreaseDefenderTactic()
    : Tactic(true, {RobotCapability::Move}),
      fsm(),
      control_params({Point(0, 0), CreaseDefenderAlignment::CENTRE})
{
}

void CreaseDefenderTactic::updateWorldParams(const World &world) {}

double CreaseDefenderTactic::calculateRobotCost(const Robot &robot,
                                                const World &world) const
{
    auto block_point = CreaseDefenderFSM::findBlockThreatPoint(
        world.field(), control_params.enemy_threat_origin,
        control_params.crease_defender_alignment);
    // Prefer robots closer to the crease defender desired position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = 1.0;
    if (block_point)
    {
        cost = (robot.position() - block_point.value()).length() /
               world.field().totalXLength();
    }
    return std::clamp<double>(cost, 0, 1);
}

void CreaseDefenderTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(false);

    do
    {
        stop_action->updateControlParams(*robot_, false);
        yield(stop_action);
    } while (!stop_action->done());
}

void CreaseDefenderTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void CreaseDefenderTactic::updateControlParams(const Point &enemy_threat_origin,
                                               const CreaseDefenderAlignment &alignment)
{
    control_params.enemy_threat_origin       = enemy_threat_origin;
    control_params.crease_defender_alignment = alignment;
}

bool CreaseDefenderTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void CreaseDefenderTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(CreaseDefenderFSM::Update(control_params, tactic_update));
}
