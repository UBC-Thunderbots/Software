#include "software/ai/hl/stp/tactic/cherry_pick_tactic.h"

#include <numeric>

#include "software/ai/hl/stp/action/move_action.h"
#include "software/geom/algorithms/distance.h"

CherryPickTactic::CherryPickTactic(const World& world, const Pass& pass)
    : Tactic(true, {RobotCapability::Move}), world_(world), pass_(pass)
{
}

void CherryPickTactic::updateWorldParams(const World& world)
{
    this->world_ = world;
}

void CherryPickTactic::updateControlParams(const Pass& pass)
{
    pass_ = pass;
}

double CherryPickTactic::calculateRobotCost(const Robot& robot, const World& world) const
{
    // Prefer robots closer to the target pass
    return distance(robot.position(), this->pass_.receiverPoint());
}

void CherryPickTactic::calculateNextAction(ActionCoroutine::push_type& yield)
{
    auto move_action = std::make_shared<MoveAction>(
        true, MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, Angle());
    do
    {
        move_action->updateControlParams(
            *robot_, pass_.receiverPoint(),
            pass_.receiverOrientation(world_.ball().position()), 0, DribblerMode::OFF,
            BallCollisionType::AVOID);
        yield(move_action);

    } while (true);
}

void CherryPickTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

World CherryPickTactic::getWorld() const
{
    return this->world_;
}
