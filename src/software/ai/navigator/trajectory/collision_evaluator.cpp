#include "software/ai/navigator/trajectory/collision_evaluator.h"

const double FRONT_COLLISION_COST_CONST = 3.0;
const double BACK_COLLISION_COST_CONST  = 1.0;
const double MID_TRAJ_COST_CONST        = 6.0;
CollisionEvaluator::CollisionEvaluator(const std::vector<ObstaclePtr> &obstacles)
    : obstacles(obstacles)
{
}

TrajectoryPathWithCost CollisionEvaluator::evaluate(
    const TrajectoryPath &trajectory,
    const std::optional<TrajectoryPathWithCost> &sub_traj_with_cost,
    const std::optional<double> sub_traj_duration_s, const std::optional<double> max_cost)
{
    TrajectoryPathWithCost traj_with_cost(trajectory);

    const double traj_time = trajectory.getTotalTime();
    const double search_end_time_s =
        std::min(trajectory.getTotalTime(), MAX_FUTURE_COLLISION_CHECK_SEC);
    const Point destination = traj_with_cost.traj_path.getDestination();

    // Initialize total cost as trajectory time
    double total_cost = traj_time;


    // Find the start duration before the trajectory leaves all obstacles
    double first_non_collision_time;
    // Avoid finding the first non-collision time if the cache sub-trajectory
    // has a collision time.
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

    // Add first front collision to total cost
    total_cost += FRONT_COLLISION_COST_CONST * first_non_collision_time;

    // Return early if current cost already higher than max cost
    if (total_cost >= max_cost)
    {
        traj_with_cost.cost = total_cost;
        return traj_with_cost;
    }

    // Find the duration we're within an obstacle before search_end_time_s
    double last_non_collision_time =
        getLastNonCollisionTime(trajectory, search_end_time_s);
    traj_with_cost.collision_duration_back_s =
        search_end_time_s - last_non_collision_time;

    // Add back collision to total cost
    total_cost += BACK_COLLISION_COST_CONST * traj_with_cost.collision_duration_back_s;

    // Return early if current cost already higher than max cost
    if (total_cost >= max_cost)
    {
        traj_with_cost.cost = total_cost;
        return traj_with_cost;
    }


    // Get the first collision time, excluding the time at the start and end of path
    // that we may be in an obstacle for.
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

    // Add 6.0 to collision if mid-trajectory collision exist
    if (traj_with_cost.colliding_obstacle != nullptr)
    {
        total_cost += MID_TRAJ_COST_CONST;
    }

    //  Add distance from first collision to destination
    Point first_collision_position =
        trajectory.getPosition(traj_with_cost.first_collision_time_s);
    total_cost += (first_collision_position - destination).length();

    //  Add early collision penalty
    total_cost += std::max(
        0.0, MAX_FUTURE_COLLISION_CHECK_SEC - traj_with_cost.first_collision_time_s);

    traj_with_cost.cost = total_cost;
    return traj_with_cost;
}


double CollisionEvaluator::getFirstNonCollisionTime(const TrajectoryPath &traj_path,
                                                    const double search_end_time_s) const
{
    double path_duration = traj_path.getTotalTime();
    for (double time = 0.0; time <= search_end_time_s;
         time += FORWARD_COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position       = traj_path.getPosition(time);
        bool collision_found = false;
        for (const ObstaclePtr &obstacle : obstacles)
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

std::pair<double, ObstaclePtr> CollisionEvaluator::getFirstCollisionTime(
    const TrajectoryPath &traj_path, const double start_time_s,
    const double search_end_time_s) const
{
    for (double time = start_time_s; time <= search_end_time_s;
         time += COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position = traj_path.getPosition(time);
        for (const ObstaclePtr &obstacle : obstacles)
        {
            if (obstacle->contains(position, time))
            {
                return std::make_pair(time, obstacle);
            }
        }
    }

    // No collision found
    return std::make_pair(std::numeric_limits<double>::max(), nullptr);
}

double CollisionEvaluator::getLastNonCollisionTime(const TrajectoryPath &traj_path,
                                                   const double search_end_time_s) const
{
    for (double time = search_end_time_s; time >= 0.0;
         time -= COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position       = traj_path.getPosition(time);
        bool collision_found = false;
        for (const ObstaclePtr &obstacle : obstacles)
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
