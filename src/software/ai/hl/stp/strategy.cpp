#include "software/ai/hl/stp/strategy.h"

Pose Strategy::getBestDribblePose(const Robot& robot)
{
    if (robot_to_best_dribble_location_.contains(robot))
    {
        return robot_to_best_dribble_location_.at(robot);
    }

    // TODO(#3082): find best dribble_position

    // cache the dribble position
    // robot_to_best_dribble_location_[robot] = ...

    // return robot_to_best_dribble_location_.at(robot)
    return Pose();
}

Pass Strategy::getBestPass(const Robot& robot)
{
    if (robot_to_best_pass_.contains(robot))
    {
        return robot_to_best_pass_.at(robot);
    }

    // calculate best pass

    return Pass(Point(), Point(), 1);
}

void Strategy::reset()
{
    robot_to_best_dribble_location_ = {};
    robot_to_best_pass_ = {};
}
