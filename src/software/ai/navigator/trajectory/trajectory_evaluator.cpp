#include "software/ai/navigator/trajectory/trajectory_evaluator.h"

#include <algorithm>

TrajectoryEvaluator::TrajectoryEvaluator(const std::vector<ObstaclePtr>& obstacles)
    : obstacles(obstacles)
{
}

TrajectoryPathWithCost TrajectoryEvaluator::evaluate(
    const TrajectoryPath& trajectory,
    const std::optional<TrajectoryPathWithCost>& sub_traj_with_cost,
    const std::optional<double> sub_traj_duration_s, const double max_cost)
{
    TrajectoryPathWithCost traj_with_cost(trajectory);

    double total_cost = 0;
    total_cost += evaluateCollisions(trajectory, traj_with_cost, sub_traj_with_cost,
                                     sub_traj_duration_s, max_cost);
    total_cost += evaluatePath(trajectory);

    traj_with_cost.cost = total_cost;
    return traj_with_cost;
}

double TrajectoryEvaluator::evaluateCollisions(
    const TrajectoryPath& trajectory, TrajectoryPathWithCost& traj_with_cost,
    const std::optional<TrajectoryPathWithCost>& sub_traj_with_cost,
    const std::optional<double> sub_traj_duration_s, const double max_cost)
{
    const double traj_time = trajectory.getTotalTime();
    const double search_end_time_s =
        std::min(trajectory.getTotalTime(), MAX_FUTURE_COLLISION_CHECK_SEC);
    const Point destination = traj_with_cost.traj_path.getDestination();

    double total_cost = traj_time;

    double first_non_collision_time;
    if (sub_traj_with_cost.has_value() &&
        sub_traj_with_cost->collision_duration_front_s < sub_traj_duration_s)
    {
        first_non_collision_time = sub_traj_with_cost->collision_duration_front_s;
    }
    else
    {
        first_non_collision_time =
            getFirstNonCollisionTime(trajectory, search_end_time_s);
    }
    traj_with_cost.collision_duration_front_s = first_non_collision_time;

    total_cost += FRONT_COLLISION_COST_CONST * first_non_collision_time;

    if (total_cost >= max_cost)
    {
        return total_cost;
    }

    double last_non_collision_time =
        getLastNonCollisionTime(trajectory, search_end_time_s);
    traj_with_cost.collision_duration_back_s =
        search_end_time_s - last_non_collision_time;

    total_cost += BACK_COLLISION_COST_CONST * traj_with_cost.collision_duration_back_s;

    if (total_cost >= max_cost)
    {
        return total_cost;
    }

    if (sub_traj_with_cost.has_value() &&
        sub_traj_with_cost->first_collision_time_s < sub_traj_duration_s)
    {
        traj_with_cost.first_collision_time_s =
            sub_traj_with_cost->first_collision_time_s;
        traj_with_cost.colliding_obstacle = sub_traj_with_cost->colliding_obstacle;
    }
    else
    {
        std::pair<double, ObstaclePtr> collision = getFirstCollisionTime(
            trajectory, first_non_collision_time, last_non_collision_time);
        traj_with_cost.first_collision_time_s = collision.first;
        traj_with_cost.colliding_obstacle     = collision.second;
    }

    if (traj_with_cost.colliding_obstacle != nullptr)
    {
        total_cost += MID_TRAJ_COST_CONST;
    }

    Point first_collision_position =
        trajectory.getPosition(traj_with_cost.first_collision_time_s);
    total_cost += (first_collision_position - destination).length();

    total_cost += std::max(
        0.0, MAX_FUTURE_COLLISION_CHECK_SEC - traj_with_cost.first_collision_time_s);

    return total_cost;
}

double TrajectoryEvaluator::evaluatePath(const TrajectoryPath& trajectory)
{
    return 0.0;
}

double TrajectoryEvaluator::evaluateDestinationSimilarity(
    const TrajectoryPath& new_trajectory, const TrajectoryPath& current_trajectory)
{
    const Point new_first_dest =
        new_trajectory.getTrajectoryPathNodes()[0].getTrajectory()->getDestination();
    const Point cur_first_dest =
        current_trajectory.getTrajectoryPathNodes()[0].getTrajectory()->getDestination();
    return (new_first_dest - cur_first_dest).length();
}

double TrajectoryEvaluator::getFirstNonCollisionTime(const TrajectoryPath& traj_path,
                                                     const double search_end_time_s) const
{
    const double path_duration = traj_path.getTotalTime();
    for (double time = 0.0; time <= search_end_time_s;
         time += FORWARD_COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position       = traj_path.getPosition(time);
        bool collision_found = false;
        for (const ObstaclePtr& obstacle : obstacles)
        {
            if (obstacle->contains(position, time))
            {
                collision_found = true;
                break;
            }
        }

        if (!collision_found)
        {
            return time;
        }
    }
    return path_duration;
}

std::pair<double, ObstaclePtr> TrajectoryEvaluator::getFirstCollisionTime(
    const TrajectoryPath& traj_path, const double start_time_s,
    const double search_end_time_s) const
{
    for (double time = start_time_s; time <= search_end_time_s;
         time += COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position = traj_path.getPosition(time);
        for (const ObstaclePtr& obstacle : obstacles)
        {
            if (obstacle->contains(position, time))
            {
                return std::make_pair(time, obstacle);
            }
        }
    }
    return std::make_pair(std::numeric_limits<double>::max(), nullptr);
}

double TrajectoryEvaluator::getLastNonCollisionTime(const TrajectoryPath& traj_path,
                                                    const double search_end_time_s) const
{
    for (double time = search_end_time_s; time >= 0.0;
         time -= COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position       = traj_path.getPosition(time);
        bool collision_found = false;
        for (const ObstaclePtr& obstacle : obstacles)
        {
            if (obstacle->contains(position, time))
            {
                collision_found = true;
                break;
            }
        }

        if (!collision_found)
        {
            return time;
        }
    }
    return search_end_time_s;
}
