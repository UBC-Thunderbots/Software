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

    // Destination similarity thresholds — trajectories within these distances are
    // considered to have the same destination and the old trajectory is kept.
    static constexpr double LINEAR_DESTINATION_THRESHOLD_METERS   = 0.03;
    static constexpr double ANGULAR_DESTINATION_THRESHOLD_DEGREES = 4.0;

    // Default threshold for switching to a new trajectory based on first sub-destination
    // distance. Goalie uses a tighter value for more precise positioning.
    static constexpr double DESTINATION_SIMILARITY_THRESHOLD_M        = 0.15;
    static constexpr double GOALIE_DESTINATION_SIMILARITY_THRESHOLD_M = 0.02;


    /**
     * @param obstacles Obstacles to check collisions against
     */
    explicit TrajectoryEvaluator(const std::vector<ObstaclePtr>& obstacles);

    /**
     * Evaluates a trajectory for collision information and cost, optionally reusing
     * collision results from a previously evaluated prefix trajectory.
     *
     * @param trajectory The trajectory to evaluate.
     * @param sub_traj_with_cost Optional prefix trajectory whose collision info may be
     * reused if it fully covers the relevant interval.
     * @param sub_traj_duration_s Duration (in seconds) of the prefix within trajectory.
     * @param max_cost Current best (minimum) cost — used for early termination.
     * @return TrajectoryPathWithCost with collision info and total cost.
     */
    TrajectoryPathWithCost evaluate(
        const TrajectoryPath& trajectory,
        const std::optional<TrajectoryPathWithCost>& sub_traj_with_cost,
        std::optional<double> sub_traj_duration_s, double max_cost);

    /**
     * Returns the distance between the first sub-destination of new_trajectory and the
     * first sub-destination of current_trajectory. A small value means both plans route
     * through the same intermediate point; a large value means the new plan takes a
     * meaningfully different path.
     *
     * Called on the winning trajectory after the search so that candidate selection
     * remains purely collision/time driven.
     *
     * @param new_trajectory Best trajectory selected by the planner this tick.
     * @param current_trajectory Trajectory the robot is currently executing.
     */
    static double evaluateDestinationSimilarity(const TrajectoryPath& new_trajectory,
                                                const TrajectoryPath& current_trajectory);

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

    double getFirstNonCollisionTime(const TrajectoryPath& traj_path,
                                    double search_end_time_s) const;

    std::pair<double, ObstaclePtr> getFirstCollisionTime(const TrajectoryPath& traj_path,
                                                         double start_time_s,
                                                         double search_end_time_s) const;

    double getLastNonCollisionTime(const TrajectoryPath& traj_path,
                                   double search_end_time_s) const;

    std::vector<ObstaclePtr> obstacles;
};
