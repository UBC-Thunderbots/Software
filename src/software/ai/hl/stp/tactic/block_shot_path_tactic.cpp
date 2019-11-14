#include "software/ai/hl/stp/tactic/block_shot_path_tactic.h"

#include <g3log/g3log.hpp>

#include "shared/constants.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/geom/util.h"

BlockShotPathTactic::BlockShotPathTactic(const Field& field, bool loop_forever)
    : Tactic(loop_forever), field(field)
{
}

std::string BlockShotPathTactic::getName() const
{
    return "Block Shot Path Tactic";
}

void BlockShotPathTactic::updateControlParams(const Robot& enemy_robot)
{
    updateControlParams(enemy_robot.position());
}

void BlockShotPathTactic::updateControlParams(const Point& shot_origin)
{
    // Update the control parameters stored by this Tactic
    this->shot_origin = shot_origin;
}

double BlockShotPathTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the block position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    Point block_position = getBlockPosition();
    double cost =
        (robot.position() - block_position).length() / world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

Point BlockShotPathTactic::getBlockPosition()
{
    Point block_position = calcBlockCone(this->field.friendlyGoalpostPos(),
                                         this->field.friendlyGoalpostNeg(),
                                         this->shot_origin, ROBOT_MAX_RADIUS_METERS);
    return block_position;
}

void BlockShotPathTactic::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    MoveAction move_action = MoveAction(0, Angle(), false);
    do
    {
        Point block_position = getBlockPosition();
        // We want to face the shot
        Angle block_orientation = (this->shot_origin - block_position).orientation();
        move_action.updateControlParams(*robot, block_position, block_orientation, 0.0,
                                        DribblerEnable::OFF, MoveType::NORMAL,
                                        AutokickType::NONE, BallCollisionType::ALLOW);
        yield(move_action.getNextIntent());
    } while (!move_action.done());
}

void BlockShotPathTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}
