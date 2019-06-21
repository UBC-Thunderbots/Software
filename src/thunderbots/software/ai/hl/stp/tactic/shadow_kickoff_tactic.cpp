#include "ai/hl/stp/tactic/shadow_kickoff_tactic.h"

#include <algorithm>

#include "ai/hl/stp/action/move_action.h"
#include "shared/constants.h"

ShadowKickoffTactic::ShadowKickoffTactic(const Field& field, bool loop_forever)
    : field(field), Tactic(loop_forever)
{
}

std::string ShadowKickoffTactic::getName() const
{
    return "ShadowKickoff Tactic";
}

void ShadowKickoffTactic::updateParams(Point enemy_robot_position)
{
    // Update the parameters stored by this Tactic
    this->enemy_robot_position = enemy_robot_position;
}

double ShadowKickoffTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the destination
    // We normalize with the diagonal so that the cost of a robot to shadow another robot
    // that is one corner, to the other corner on the diagonal is 1
    Vector diagonal(world.field().totalWidth(), world.field().totalLength());
    double cost = (robot.position() - this->enemy_robot_position).len() / diagonal.len();
    return std::clamp<double>(cost, 0, 1);
}

void ShadowKickoffTactic::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    MoveAction move_action = MoveAction();
    do
    {
        yield(move_action.updateStateAndGetNextIntent(
            *robot, this->calculateShadowPoint(field, enemy_robot_position),
            Angle::half(), 0));
    } while (!move_action.done());
}

Point ShadowKickoffTactic::calculateShadowPoint(const Field& field,
                                                const Point& enemy_robot_position)
{
    Point destination;

    if (std::fabs(enemy_robot_position.y()) <
        (field.centreCircleRadius() + 4 * ROBOT_MAX_RADIUS_METERS))
    {
        destination = Point(-(field.centreCircleRadius() + 2 * ROBOT_MAX_RADIUS_METERS),
                            enemy_robot_position.y());
    }
    else
    {
        destination = Point(-2 * ROBOT_MAX_RADIUS_METERS, enemy_robot_position.y());
    }

    return destination;
}
