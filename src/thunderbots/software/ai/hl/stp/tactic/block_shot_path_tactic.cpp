#include "ai/hl/stp/tactic/block_shot_path_tactic.h"

#include "geom/util.h"
#include "shared/constants.h"
#include "util/logger/init.h"

BlockShotPathTactic::BlockShotPathTactic(const Field& field, bool loop_forever)
    : field(field), Tactic(loop_forever)
{
}

std::string BlockShotPathTactic::getName() const
{
    return "Block Shot Path Tactic";
}

void BlockShotPathTactic::updateParams(const Robot& enemy_robot)
{
    updateParams(enemy_robot.position());
}

void BlockShotPathTactic::updateParams(const Point& shot_origin)
{
    // Update the parameters stored by this Tactic
    this->shot_origin = shot_origin;
}

double BlockShotPathTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the block position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    Point block_position = getBlockPosition();
    double cost = (robot.position() - block_position).len() / world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

Point BlockShotPathTactic::getBlockPosition()
{
    Point block_position = calcBlockCone(this->field.friendlyGoalpostPos(),
                                         this->field.friendlyGoalpostNeg(),
                                         this->shot_origin, ROBOT_MAX_RADIUS_METERS);
    return block_position;
}

std::unique_ptr<Intent> BlockShotPathTactic::calculateNextIntent(
    intent_coroutine::push_type& yield)
{
    MoveAction move_action = MoveAction();
    do
    {
        Point block_position = getBlockPosition();
        // We want to face the shot
        Angle block_orientation = (this->shot_origin - block_position).orientation();
        yield(move_action.updateStateAndGetNextIntent(*robot, block_position,
                                                      block_orientation, 0.0));
    } while (!move_action.done());
}
