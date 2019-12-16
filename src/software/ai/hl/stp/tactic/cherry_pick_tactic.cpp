/**
 * Definition for the CherryPickTactic class
 */

#include "software/ai/hl/stp/tactic/cherry_pick_tactic.h"

#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/geom/util.h"

CherryPickTactic::CherryPickTactic(const World& world, const Rectangle& target_region)
    : Tactic(true),
      pass_generator(world, world.ball().position(), Passing::PassType::ONE_TOUCH_SHOT),
      world(world),
      target_region(target_region)
{
    pass_generator.setTargetRegion(target_region);
}

std::string CherryPickTactic::getName() const
{
    return "Cherry Pick Tactic";
}

void CherryPickTactic::updateWorldParams(const World& world)
{
    this->world = world;
}

double CherryPickTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the target region
    return dist(robot.position(), target_region);
}

void CherryPickTactic::calculateNextAction(ActionCoroutine::push_type& yield)
{
    auto move_action = std::make_shared<MoveAction>(
        MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, Angle(), true);
    auto best_pass_and_score = pass_generator.getBestPassSoFar();
    do
    {
        pass_generator.setWorld(world);
        // Move the robot to be the best possible receiver for the best pass we can
        // find (within the target region)
        Pass pass = pass_generator.getBestPassSoFar().pass;
        move_action->updateControlParams(*robot, pass.receiverPoint(),
                                         pass.receiverOrientation(), 0,
                                         DribblerEnable::OFF, MoveType::NORMAL,
                                         AutokickType::NONE, BallCollisionType::AVOID);
        yield(move_action);
    } while (true);
}

void CherryPickTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}
