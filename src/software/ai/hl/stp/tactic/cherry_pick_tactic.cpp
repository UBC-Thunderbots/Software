#include "software/ai/hl/stp/tactic/cherry_pick_tactic.h"

#include "software/ai/hl/stp/action/move_action.h"
#include "software/geom/algorithms/distance.h"

CherryPickTactic::CherryPickTactic(const World& world, const Rectangle& target_region, std::shared_ptr<const PassingConfig> passing_config)
    : Tactic(true, {RobotCapability::Move}),
      pass_generator(world, world.ball().position(), PassType::ONE_TOUCH_SHOT, passing_config),
      world(world),
      target_region(target_region)
{
    pass_generator.setTargetRegion(target_region);
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
    auto move_action = std::make_shared<MoveAction>(
        true, MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, Angle());
    auto best_pass_and_score = pass_generator.getBestPassSoFar();
    do
    {
        pass_generator.setWorld(world);
        // Move the robot to be the best possible receiver for the best pass we can
        // find (within the target region)
        Pass pass = pass_generator.getBestPassSoFar().pass;
        move_action->updateControlParams(*robot_, pass.receiverPoint(),
                                         pass.receiverOrientation(), 0, DribblerMode::OFF,
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
    return this->world;
}
