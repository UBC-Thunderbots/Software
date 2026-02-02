#pragma once

#include <optional>

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
     * @param prev_sub_destination The previous sub destination of this robot.
     * nullopt if there is no previous sub destination
     * @return TrajectoryPath which attempts to avoid the obstacles
     */
    std::optional<TrajectoryPath> findTrajectory(
        const Point &start, const Point &destination, const Vector &initial_velocity,
        const KinematicConstraints &constraints,
        const std::vector<ObstaclePtr> &obstacles, const Rectangle &navigable_area,
        const std::optional<Point> &prev_sub_destination = std::nullopt);

   private:
    /**
     * Get a single trajectory with cost that goes directly from the start to the
     * destination.
     *
     * @param start Start position of the trajectory
     * @param destination Destination of the trajectory
     * @param initial_velocity Initial velocity of the trajectory
     * @param constraints Kinematic constraints of the trajectory
     * @param obstacles List of all obstacles
     * @return A trajectory path with only a single trajectory + its cost
     */
    TrajectoryPathWithCost getDirectTrajectoryWithCost(
        const Point &start, const Point &destination, const Vector &initial_velocity,
        const KinematicConstraints &constraints,
        const std::vector<ObstaclePtr> &obstacles);

    /**
     * Given a trajectory path, calculate its cost
     *
     * @param trajectory The trajectory path to calculate the cost of
     * @param obstacles List of all obstacles
     * @param sub_traj_with_cost Optional cached trajectory path with cost of the sub
     * trajectory
     * @param sub_traj_duration_s Optional duration of the cached sub_traj_with_cost
	 * @param max_cost Current maximum cost among calculated trajectories
     * @return The trajectory path with its cost
     */
    TrajectoryPathWithCost getTrajectoryWithCost(
        const TrajectoryPath &trajectory, const std::vector<ObstaclePtr> &obstacles,
        const std::optional<TrajectoryPathWithCost> &sub_traj_with_cost,
        const std::optional<double> sub_traj_duration_s,
		const std::optional<double> max_cost);



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
    static constexpr std::array<double, 4> SUB_DESTINATION_DISTANCES_METERS = {0.4, 1.1,
                                                                               2.3, 3};
    static constexpr unsigned int NUM_SUB_DESTINATION_ANGLES                = 16;
    static constexpr Angle MIN_SUB_DESTINATION_ANGLE = Angle::fromDegrees(20);
    static constexpr Angle MAX_SUB_DESTINATION_ANGLE = Angle::fromDegrees(140);

    const double SUB_DESTINATION_STEP_INTERVAL_SEC = 0.2;


    const double SUB_DESTINATION_CLOSE_BONUS_THRESHOLD_METERS = 0.1;
    const double SUB_DESTINATION_CLOSE_BONUS_COST             = -0.3;
};
