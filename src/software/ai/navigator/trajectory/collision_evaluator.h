#pragma once

#include <optional>

#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/ai/navigator/trajectory/trajectory_path_with_cost.h"

class CollisionEvaluator
{
  public:
    explicit CollisionEvaluator(const std::vector<ObstaclePtr>& obstacles);

  TrajectoryPathWithCost evaluate(
    const TrajectoryPath &trajectory,
    const std::optional<TrajectoryPathWithCost> &sub_traj_with_cost,
    std::optional<double> sub_traj_duration_s
  );
    private:
    std::vector<ObstaclePtr> obstacles;

    /**
     * Get the earliest time at which the trajectory is not in a collision, in seconds
     * E.g. will return 0 if the trajectory's start position is not in an obstacle
     *
     * @param traj_path The trajectory path to check
     * @param search_end_time_s The latest time to check for collisions
     * @return Earliest non-collision time, or traj_path.getTotalDuration() if the
     * trajectory is in a collision from start to search_end_time_s
     */
    double getFirstNonCollisionTime(const TrajectoryPath &traj_path,
                                    const double search_end_time_s) const;

    /**
     * Find if there was a collision between the start_time_sec and search_end_time_s
     * for the given trajectory path and obstacles.
     *
     * @param traj_path The trajectory path to check
     * @param start_time_s The time in seconds to start the search from
     * @param search_end_time_s The time in seconds to stop the search at
     * @return The first collision time within [start_time_sec and search_end_time_s]
     * and a pointer to the obstacle if a collision exists, otherwise returns
     * std::numeric_limits<double>::max() and nullptr.
     */
    std::pair<double, ObstaclePtr> getFirstCollisionTime(
        const TrajectoryPath &traj_path,
        const double start_time_s, const double search_end_time_s) const;

    /**
     * Returns the latest time (within the search_end_time_s) at which the trajectory
     * is NOT in a collision. Will return search_end_time_s if the trajectory does not
     * end in a collision.
     *
     * @param traj_path The trajectory path to check
     * @param search_end_time_s The latest time to check for collisions. Assumed to
     * be within the duration of the trajectory path.
     * @return Time in seconds at which the trajectory is not in a collision. Result
     * will be in the range [0, search_end_time_s].
     */
    double getLastNonCollisionTime(const TrajectoryPath &traj_path,
                                   const double search_end_time_s) const;

};

const double COLLISION_CHECK_STEP_INTERVAL_SEC         = 0.1;
const double FORWARD_COLLISION_CHECK_STEP_INTERVAL_SEC = 0.05;
const double MAX_FUTURE_COLLISION_CHECK_SEC            = 2.0;
