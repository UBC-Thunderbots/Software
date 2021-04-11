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

CreaseDefenderTactic::CreaseDefenderTactic(
    CreaseDefenderAlignment crease_defender_alignment)
    : Tactic(true, {RobotCapability::Move}), fsm(), control_params()
{
    control_params.crease_defender_alignment = crease_defender_alignment;
}

void CreaseDefenderTactic::updateWorldParams(const World &world) {}

double CreaseDefenderTactic::calculateRobotCost(const Robot &robot,
                                                const World &world) const
{
    // Prefer robots closer to the crease defender desired position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    // TODO: fix this hack for the robot cost
    double cost = 0;
    cost = (robot.position() - world.field().friendlyDefenseArea().posXPosYCorner())
               .length() /
           world.field().totalXLength();
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

void CreaseDefenderTactic::updateIntent(const TacticUpdate &tactic_update)
{
    control_params.enemy_threat_origin = tactic_update.world.ball().position();
    fsm.process_event(CreaseDefenderFSM::Update(control_params, tactic_update));
}
