#include "software/ai/hl/stp/strategy/strategy.h"

Pose Strategy::getBestDribblePose(const Robot& robot)
{
    if (robot_to_best_dribble_location_.contains(robot.id()))
    {
        return robot_to_best_dribble_location_.at(robot.id());
    }

    // TODO(#3082): find best dribble_position

    // cache the dribble position
    // robot_to_best_dribble_location_[robot] = ...

    // return robot_to_best_dribble_location_.at(robot)
    return Pose();
}

Pass Strategy::getBestPass(const Robot& robot)
{
    if (robot_to_best_pass_.contains(robot.id()))
    {
        return robot_to_best_pass_.at(robot.id());
    }

    // calculate best pass

    return Pass(Point(), Point(), 1);
}

std::optional<Shot> Strategy::getBestShot(const Robot& robot, const World& world)
{
    if (robot_to_best_shot_.contains(robot.id()))
    {
        return robot_to_best_shot_.at(robot.id());
    }

    robot_to_best_shot_[robot.id()] =
        calcBestShotOnGoal(world.field(), world.friendlyTeam(), world.enemyTeam(),
                           robot.position(), TeamType::ENEMY, {robot});
    return robot_to_best_shot_[robot.id()];
}

std::vector<OffenseSupportType> Strategy::getCommittedOffenseSupport() const
{
    // TODO(#3098): Commit the Support tactics to this Strategy class and return their
    // types here
    return std::vector<OffenseSupportType>();
}

void Strategy::reset()
{
    robot_to_best_dribble_location_ = {};
    robot_to_best_pass_             = {};
    robot_to_best_shot_             = {};
}
