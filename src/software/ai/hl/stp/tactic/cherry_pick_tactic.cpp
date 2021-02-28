#include "software/ai/hl/stp/tactic/cherry_pick_tactic.h"

#include "software/ai/hl/stp/action/move_action.h"
#include "software/geom/algorithms/distance.h"

CherryPickTactic::CherryPickTactic(const World& world, const Rectangle& target_region, std::set<FieldZone> zones)
    : Tactic(true, {RobotCapability::Move}),
{
}

void CherryPickTactic::updateWorldParams(const World& world)
{
    this->world = world;
}

double CherryPickTactic::calculateRobotCost(const Robot& robot, const World& world) const
{
    // Prefer robots closer to the target region
    return distance(robot.position(), target_region);
}

void CherryPickTactic::calculateNextAction(ActionCoroutine::push_type& yield)
{
    pass_eval.getBestPassInZone(
}

void CherryPickTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

World CherryPickTactic::getWorld() const
{
    return this->world;
}
