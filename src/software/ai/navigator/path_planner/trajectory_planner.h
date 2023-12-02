#pragma once

#include <optional>

#include "extlibs/AABB/AABB.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/path_planner/trajectory_path.h"
#include "software/ai/navigator/path_planner/trajectory_path_with_cost.h"

class TrajectoryPlanner
{
   public:
    TrajectoryPlanner();

    TrajectoryPath findTrajectory(const Point &start, const Point &destination,
                                  const Vector &initial_velocity,
                                  const KinematicConstraints &constraints,
                                  const std::vector<ObstaclePtr> &obstacles,
                                  const Rectangle &navigable_area);

   private:
    double calculateCost(const TrajectoryPath &trajectory_path, aabb::Tree &obstacle_tree,
                         const std::vector<ObstaclePtr> &obstacles);
    TrajectoryPathWithCost getDirectTrajectoryWithCost(
        const Point &start, const Point &destination, const Vector &initial_velocity,
        const KinematicConstraints &constraints, aabb::Tree &obstacle_tree,
        const std::vector<ObstaclePtr> &obstacles);
    TrajectoryPathWithCost getTrajectoryWithCost(
        const TrajectoryPath &trajectory, aabb::Tree &obstacle_tree,
        const std::vector<ObstaclePtr> &obstacles,
        const std::optional<TrajectoryPathWithCost> &sub_traj_with_cost,
        const std::optional<double> sub_traj_duration_sec);
    double calculateCost(const TrajectoryPathWithCost &traj_with_cost) const;

    double getFirstNonCollisionTime(const TrajectoryPath &traj_path,
                                    const std::set<unsigned int> &obstacle_indices,
                                    const std::vector<ObstaclePtr> &obstacles,
                                    const double search_end_time_s) const;

    /**
     * Find if there was a collision between the start_time_sec and search_end_time_s
     * for the given trajectory path and obstacles.
     *
     * @param traj_path The trajectory path to check
     * @param obstacle_indices The indices of the obstacles to check for collisions
     * @param obstacles The list of all obstacles
     * @param start_time_sec The time in seconds to start the search from
     * @param search_end_time_s The time in seconds to stop the search at
     * @return The first collision time within [start_time_sec and search_end_time_s]
     * and a pointer to the obstacle if a collision exists, otherwise returns
     * std::numeric_limits<double>::max() and nullptr.
     */
    std::pair<double, ObstaclePtr> getFirstCollisionTime(
        const TrajectoryPath &traj_path, const std::set<unsigned int> &obstacle_indices,
        const std::vector<ObstaclePtr> &obstacles,
        const double start_time_sec,  // TODO: Somewhere we use sec, somewhere we use _s
        const double search_end_time_s) const;

    /**
     * Returns the latest time (within the search_end_time_s) at which the trajectory
     * is NOT in a collision. Will return search_end_time_s if the trajectory does not
     * end in a collision.
     *
     * @param traj_path The trajectory path to check
     * @param obstacle_indices The indices of the obstacles to check for collisions
     * @param obstacles The list of all obstacles
     * @param search_end_time_s The latest time to check for collisions. Assumed to
     * be within the duration of the trajectory path.
     * @return Time in seconds at which the trajectory is not in a collision. Result
     * will be in the range [0, search_end_time_s].
     */
    double getLastNonCollisionTime(const TrajectoryPath &traj_path,
                                   const std::set<unsigned int> &obstacle_indices,
                                   const std::vector<ObstaclePtr> &obstacles,
                                   const double search_end_time_s) const;

    std::vector<Vector> relative_sub_destinations;
    std::optional<Point> last_sub_dest;

    TrajectoryGenerator trajectory_generator;

    const double LAST_SUB_DESTINATION_BONUS             = 0.5;
    const double LAST_SUB_DESTINATION_STEP_INTERVAL_SEC = 0.05;

    const double SUB_DESTINATION_STEP_INTERVAL_SEC         = 0.2;
    const double COLLISION_CHECK_STEP_INTERVAL_SEC         = 0.1;
    const double FORWARD_COLLISION_CHECK_STEP_INTERVAL_SEC = 0.05;
    const double MAX_FUTURE_COLLISION_CHECK_SEC            = 2.0;

    static constexpr std::array<double, 4> SUB_DESTINATION_DISTANCES_METERS = {0.1, 1.1,
                                                                               2.1, 3};
    static constexpr unsigned int NUM_SUB_DESTINATION_ANGLES                = 16;
};
