#pragma once

#include <optional>

#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/ai/navigator/trajectory/trajectory_path_with_cost.h"

/**
 * TrajectoryEvaluator scores a trajectory against obstacles and the robot's current
 * trajectory. Computed sub-trajectories are cached to reduce redundant collision checks.
 **/
class TrajectoryEvaluator
{
   public:
    // Collision evaluation constants
    static constexpr double COLLISION_CHECK_STEP_INTERVAL_SEC         = 0.1;
    static constexpr double FORWARD_COLLISION_CHECK_STEP_INTERVAL_SEC = 0.05;
    static constexpr double MAX_FUTURE_COLLISION_CHECK_SEC            = 2.0;
    static constexpr double FRONT_COLLISION_COST_CONST                = 3.0;
    static constexpr double BACK_COLLISION_COST_CONST                 = 1.0;
    static constexpr double MID_TRAJ_COST_CONST                       = 6.0;

    // Trajectory consistency evaluation constants
    static constexpr double CONSISTENCY_CHECK_STEP_INTERVAL_S = 0.1;
    static constexpr double CONSISTENCY_CHECK_HORIZON_S        = 0.5;
    static constexpr double TRAJECTORY_CONSISTENCY_WEIGHT      = 0.3;

    /**
     * @param obstacles Obstacles to check collisions against
     */
    explicit TrajectoryEvaluator(const std::vector<ObstaclePtr>& obstacles);

    /**
     * Evaluates a trajectory for collision information and cost, optionally reusing
     * collision results from a previously evaluated prefix trajectory and adding a
     * consistency cost relative to the robot's current trajectory.
     *
     * @param trajectory The trajectory to evaluate.
     * @param sub_traj_with_cost Optional prefix trajectory whose collision info may be
     * reused if it fully covers the relevant interval.
     * @param sub_traj_duration_s Duration (in seconds) of the prefix within trajectory.
     * @param max_cost Current best (minimum) cost — used for early termination.
     * @param current_trajectory The trajectory the robot is currently executing.
     * When provided, a consistency cost is added to penalise trajectories that deviate
     * significantly from the current path in the near future.
     * @return TrajectoryPathWithCost with collision info and total cost.
     */
    TrajectoryPathWithCost evaluate(
        const TrajectoryPath& trajectory,
        const std::optional<TrajectoryPathWithCost>& sub_traj_with_cost,
        std::optional<double> sub_traj_duration_s, double max_cost,
        const std::optional<TrajectoryPath>& current_trajectory = std::nullopt);

   private:
    /**
     * Computes the collision cost component and populates collision fields on
     * traj_with_cost. Returns the total collision cost (not yet stored on the object).
     */
    double evaluateCollisions(
        const TrajectoryPath& trajectory, TrajectoryPathWithCost& traj_with_cost,
        const std::optional<TrajectoryPathWithCost>& sub_traj_with_cost,
        std::optional<double> sub_traj_duration_s, double max_cost);

    /**
     * Computes a path-shape cost component (currently unused, returns 0).
     * Extend this to penalise undesirable path geometry.
     */
    double evaluatePath(const TrajectoryPath& trajectory);

    /**
     * Computes a cost that penalises deviating from current_trajectory in the near
     * future. Samples positions at CONSISTENCY_CHECK_STEP_INTERVAL_S intervals up to
     * CONSISTENCY_CHECK_HORIZON_S and returns TRAJECTORY_CONSISTENCY_WEIGHT times the
     * sum of positional deviations. Returns 0 if current_trajectory is empty.
     *
     * @param new_trajectory Candidate trajectory being evaluated.
     * @param current_trajectory Trajectory the robot is currently executing.
     */
    double evaluateTrajectoryConsistency(
        const TrajectoryPath& new_trajectory,
        const std::optional<TrajectoryPath>& current_trajectory) const;

    double getFirstNonCollisionTime(const TrajectoryPath& traj_path,
                                    double search_end_time_s) const;

    std::pair<double, ObstaclePtr> getFirstCollisionTime(const TrajectoryPath& traj_path,
                                                         double start_time_s,
                                                         double search_end_time_s) const;

    double getLastNonCollisionTime(const TrajectoryPath& traj_path,
                                   double search_end_time_s) const;

    std::vector<ObstaclePtr> obstacles;
};
