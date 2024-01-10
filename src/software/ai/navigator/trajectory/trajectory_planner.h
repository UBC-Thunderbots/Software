#pragma once

#include <optional>

#include "extlibs/AABB/AABB.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/ai/navigator/trajectory/trajectory_path_with_cost.h"

class TrajectoryPlanner
{
   public:
    /**
     * Constructor
     */
    TrajectoryPlanner();

    /**
     * Find a trajectory from the start position to the destination which
     * attempts to avoid the list of obstacles.
     *
     * @param start Start position of the trajectory
     * @param destination Destination of the trajectory
     * @param initial_velocity Initial velocity of the trajectory
     * @param constraints Kinematic constraints of the trajectory
     * @param obstacles List of obstacles to avoid
     * @param navigable_area The navigable area of the field
     * @return TrajectoryPath which attempts to avoid the obstacles
     */
    std::optional<TrajectoryPath> findTrajectory(
        const Point &start, const Point &destination, const Vector &initial_velocity,
        const KinematicConstraints &constraints,
        const std::vector<ObstaclePtr> &obstacles, const Rectangle &navigable_area);

   private:
    /**
     * Calculate the cost of the given trajectory path with cost
     *
     * @param traj_with_cost A complete trajectory path with cost
     * @return The cost of the trajectory
     */
    double calculateCost(const TrajectoryPathWithCost &traj_with_cost) const;

    /**
     * Get a single trajectory with cost that goes directly from the start to the
     * destination.
     *
     * @param start Start position of the trajectory
     * @param destination Destination of the trajectory
     * @param initial_velocity Initial velocity of the trajectory
     * @param constraints Kinematic constraints of the trajectory
     * @param obstacle_tree Axis aligned bounding box tree of the obstacles
     * @param obstacles List of all obstacles
     * @return A trajectory path with only a single trajectory + its cost
     */
    TrajectoryPathWithCost getDirectTrajectoryWithCost(
        const Point &start, const Point &destination, const Vector &initial_velocity,
        const KinematicConstraints &constraints, aabb::Tree &obstacle_tree,
        const std::vector<ObstaclePtr> &obstacles);

    /**
     * Given a trajectory path, calculate its cost
     *
     * @param trajectory The trajectory path to calculate the cost of
     * @param obstacle_tree Axis aligned bounding box tree of the obstacles
     * @param obstacles List of all obstacles
     * @param sub_traj_with_cost Optional cached trajectory path with cost of the sub
     * trajectory
     * @param sub_traj_duration_s Optional duration of the cached sub_traj_with_cost
     * @return The trajectory path with its cost
     */
    TrajectoryPathWithCost getTrajectoryWithCost(
        const TrajectoryPath &trajectory, aabb::Tree &obstacle_tree,
        const std::vector<ObstaclePtr> &obstacles,
        const std::optional<TrajectoryPathWithCost> &sub_traj_with_cost,
        const std::optional<double> sub_traj_duration_s);

    /**
     * Get the earliest time at which the trajectory is not in a collision, in seconds
     * E.g. will return 0 if the trajectory's start position is not in an obstacle
     *
     * @param traj_path The trajectory path to check
     * @param obstacle_indices A list of indices of the obstacles which this trajectory
     * may collide with. Used to reduce the number of collision checks
     * @param obstacles A list of all obstacles
     * @param search_end_time_s The latest time to check for collisions
     * @return Earliest non-collision time, or traj_path.getTotalDuration() if the
     * trajectory is in a collision from start to search_end_time_s
     */
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
     * @param start_time_s The time in seconds to start the search from
     * @param search_end_time_s The time in seconds to stop the search at
     * @return The first collision time within [start_time_sec and search_end_time_s]
     * and a pointer to the obstacle if a collision exists, otherwise returns
     * std::numeric_limits<double>::max() and nullptr.
     */
    std::pair<double, ObstaclePtr> getFirstCollisionTime(
        const TrajectoryPath &traj_path, const std::set<unsigned int> &obstacle_indices,
        const std::vector<ObstaclePtr> &obstacles, const double start_time_s,
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

    /**
     * Get a list of sub destinations which trajectory paths should be sampled through for
     * the given start position and destination. All sub destinations will be within the
     * navigable area.
     *
     * @param start Start position of the trajectory
     * @param destination Destination of the trajectory
     * @param navigable_area The navigable area of the field
     * @return A list of sub destinations for trajectory paths be sampled through
     */
    std::vector<Point> getSubDestinations(const Point &start, const Point &destination,
                                          const Rectangle &navigable_area) const;

    /**
     * Helper function for generating the relative sub destinations
     * given the constants below.
     *
     * @return The vector of relative sub destinations
     */
    static std::vector<Vector> getRelativeSubDestinations();

    const std::vector<Vector> relative_sub_destinations;
    static constexpr std::array<double, 4> SUB_DESTINATION_DISTANCES_METERS = {1.1, 2.3,
                                                                               3};
    static constexpr unsigned int NUM_SUB_DESTINATION_ANGLES                = 16;

    static constexpr double MIN_SUB_DESTINATION_DISTANCE_M = 0.5;
    static constexpr double MAX_SUB_DESTINATION_DISTANCE_M = 1.5;
    static constexpr Angle MIN_SUB_DESTINATION_ANGLE       = Angle::fromDegrees(20);
    static constexpr Angle MAX_SUB_DESTINATION_ANGLE       = Angle::fromDegrees(140);

    const double SUB_DESTINATION_STEP_INTERVAL_SEC         = 0.2;
    const double COLLISION_CHECK_STEP_INTERVAL_SEC         = 0.1;
    const double FORWARD_COLLISION_CHECK_STEP_INTERVAL_SEC = 0.05;
    const double MAX_FUTURE_COLLISION_CHECK_SEC            = 2.0;
};
