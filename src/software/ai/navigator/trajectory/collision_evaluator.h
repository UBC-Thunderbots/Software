#pragma once

#include <optional>

#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/ai/navigator/trajectory/trajectory_path_with_cost.h"

/**
 * Collision evaluator computes the collision between a trajectory and obstacles.
 * Computed sub-trajectories are stored in a cache to reduce computational load.
 **/
class CollisionEvaluator
{
  public:
    /**
     * Constructor
     * @param obstacles A vector of obstacle pointers, where obstacles are calculated for collision
     */
    explicit CollisionEvaluator(const std::vector<ObstaclePtr>& obstacles);


    /**
 * Evaluates a trajectory for collision information and cost, optionally reusing
 * collision results from a previously evaluated prefix trajectory.
 * If a prefix trajectory is provided, collision information that occurs entirely
 * before the given prefix duration may be reused to avoid redundant collision
 * checking. Collision checks beyond the prefix duration are computed normally.
 * @param trajectory
 *        The trajectory to evaluate for collisions and cost.
 * @param sub_traj_with_cost
 *        An optional previously evaluated prefix trajectory whose collision
 *        information may be reused if the prefix fully covers the relevant
 *        collision interval.
 * @param sub_traj_duration_s
 *        The duration (in seconds) of the prefix trajectory within trajectory.
 * @return
 *         A TrajectoryPathWithCost< containing the trajectory along with
 *         computed collision timing information and total cost.
 */
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
