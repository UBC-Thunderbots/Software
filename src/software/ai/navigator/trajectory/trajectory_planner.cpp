#include "software/ai/navigator/trajectory/trajectory_planner.h"

#include "software/ai/navigator/trajectory/trajectory_evaluator.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"


TrajectoryPlanner::TrajectoryPlanner()
    : relative_sub_destinations(getRelativeSubDestinations())
{
}

std::vector<Vector> TrajectoryPlanner::getRelativeSubDestinations()
{
    std::vector<Vector> relative_sub_destinations;
    const Angle sub_angles = Angle::full() / NUM_SUB_DESTINATION_ANGLES;
    for (const double distance : SUB_DESTINATION_DISTANCES_METERS)
    {
        for (unsigned int i = 0; i < NUM_SUB_DESTINATION_ANGLES; ++i)
        {
            Angle angle = sub_angles * i;
            relative_sub_destinations.emplace_back(
                Vector::createFromAngle(angle).normalize(distance));
        }
    }
    return relative_sub_destinations;
}

std::vector<Point> TrajectoryPlanner::getSubDestinations(
    const Point& start, const Point& destination, const Rectangle& navigable_area) const
{
    std::vector<Point> sub_destinations;
    Angle direction = (destination - start).orientation();
    sub_destinations.reserve(relative_sub_destinations.size());

    for (const Vector& relative_sub_dest : relative_sub_destinations)
    {
        Angle sub_dest_angle_to_dest = relative_sub_dest.orientation().minDiff(direction);
        if (sub_dest_angle_to_dest < MIN_SUB_DESTINATION_ANGLE ||
            sub_dest_angle_to_dest > MAX_SUB_DESTINATION_ANGLE)
        {
            continue;
        }

        Point sub_dest = start + relative_sub_dest;
        if (!contains(navigable_area, sub_dest))
        {
            continue;
        }
        sub_destinations.emplace_back(sub_dest);
    }
    return sub_destinations;
}

std::optional<TrajectoryPath> TrajectoryPlanner::findTrajectory(
    const Point& start, const Point& destination, const Vector& initial_velocity,
    const KinematicConstraints& constraints, const std::vector<ObstaclePtr>& obstacles,
    const Rectangle& navigable_area, const std::optional<Point>& prev_sub_destination,
    const std::optional<TrajectoryPath>& current_trajectory,
    double destination_similarity_threshold)
{
    if (constraints.getMaxVelocity() <= 0.0 || constraints.getMaxAcceleration() <= 0.0 ||
        constraints.getMaxDeceleration() <= 0.0)
    {
        return std::nullopt;
    }

    // Re-evaluate the current trajectory against this tick's obstacles to decide
    // whether we must replan or can just check for a better route.
    bool must_replan = true;
    if (current_trajectory.has_value())
    {
        TrajectoryEvaluator cur_evaluator(obstacles);
        TrajectoryPathWithCost current_eval = cur_evaluator.evaluate(
            current_trajectory.value(), std::nullopt, std::nullopt,
            std::numeric_limits<double>::max());
        must_replan = current_eval.cost > CURRENT_TRAJ_REPLAN_THRESHOLD;
    }

    TrajectoryPathWithCost best_traj_with_cost =
        getDirectTrajectoryWithCost(start, destination, initial_velocity, constraints,
                                    obstacles);

    // Return direct trajectory to the destination if it doesn't have any collisions
    if (!best_traj_with_cost.collides())
    {
        return best_traj_with_cost.traj_path;
    }

    // Sample trajectory paths by trying different sub destinations and connection times
    // and store the best trajectory path (min cost)
    for (const Point& sub_dest : getSubDestinations(start, destination, navigable_area))
    {
        TrajectoryPathWithCost sub_trajectory =
            getDirectTrajectoryWithCost(start, sub_dest, initial_velocity, constraints,
                                        obstacles);

        // Prefer sub destinations that are closer to the previous sub destination to
        // avoid oscillation between two sub destinations with similar cost.
        double cost_offset = 0.0;
        if (prev_sub_destination.has_value() &&
            distance(sub_dest, prev_sub_destination.value()) <
                SUB_DESTINATION_CLOSE_BONUS_THRESHOLD_METERS)
        {
            cost_offset = SUB_DESTINATION_CLOSE_BONUS_COST;
        }

        for (double connection_time = SUB_DESTINATION_STEP_INTERVAL_SEC;
             connection_time <= sub_trajectory.traj_path.getTotalTime();
             connection_time += SUB_DESTINATION_STEP_INTERVAL_SEC)
        {
            TrajectoryPath traj_path_to_dest = sub_trajectory.traj_path;
            traj_path_to_dest.append(connection_time, destination, constraints);

            if (traj_path_to_dest.getTotalTime() >= best_traj_with_cost.cost)
            {
                break;
            }

            TrajectoryPathWithCost full_traj_with_cost =
                getTrajectoryWithCost(traj_path_to_dest, obstacles, sub_trajectory,
                                      connection_time, best_traj_with_cost.cost);
            full_traj_with_cost.cost += cost_offset;
            if (full_traj_with_cost.cost < best_traj_with_cost.cost)
            {
                best_traj_with_cost = full_traj_with_cost;
            }

            if (!full_traj_with_cost.collides() ||
                (!best_traj_with_cost.collides() &&
                 full_traj_with_cost.traj_path.getTotalTime() >
                     best_traj_with_cost.traj_path.getTotalTime()))
            {
                break;
            }
        }
    }

    if (must_replan)
    {
        // Current trajectory was blocked — use the new best path unconditionally.
        LOG(INFO) << "Replanning: current trajectory exceeded cost threshold";
    }
    else
    {
        // Current trajectory is still valid — only switch if the new plan takes a
        // meaningfully different route (different first sub-destination).
        double dest_dist = TrajectoryEvaluator::evaluateDestinationSimilarity(
            best_traj_with_cost.traj_path, current_trajectory.value());
        if (dest_dist < destination_similarity_threshold)
        {
            LOG(INFO) << "Holding current trajectory (destination distance: " << dest_dist
                      << " m)";
            return current_trajectory;
        }
        LOG(INFO) << "Switching trajectory (destination distance: " << dest_dist << " m)";
    }

    double collision_velocity =
        best_traj_with_cost.traj_path
            .getVelocity(best_traj_with_cost.first_collision_time_s)
            .length();
    if (best_traj_with_cost.collides() &&
        best_traj_with_cost.first_collision_time_s <
            UNAVOIDABLE_COLLISION_TIME_THRESHOLD_S &&
        collision_velocity > UNAVOIDABLE_COLLISION_VELOCITY_THRESHOLD_M_S)
    {
        return std::nullopt;
    }

    return best_traj_with_cost.traj_path;
}

TrajectoryPathWithCost TrajectoryPlanner::getDirectTrajectoryWithCost(
    const Point& start, const Point& destination, const Vector& initial_velocity,
    const KinematicConstraints& constraints, const std::vector<ObstaclePtr>& obstacles)
{
    return getTrajectoryWithCost(
        TrajectoryPath(std::make_shared<BangBangTrajectory2D>(
                           start, destination, initial_velocity, constraints),
                       BangBangTrajectory2D::generator),
        obstacles, std::nullopt, std::nullopt, std::numeric_limits<double>::max());
}

TrajectoryPathWithCost TrajectoryPlanner::getTrajectoryWithCost(
    const TrajectoryPath& trajectory, const std::vector<ObstaclePtr>& obstacles,
    const std::optional<TrajectoryPathWithCost>& sub_traj_with_cost,
    const std::optional<double> sub_traj_duration_s, double max_cost)
{
    TrajectoryEvaluator evaluator(obstacles);
    return evaluator.evaluate(trajectory, sub_traj_with_cost, sub_traj_duration_s,
                              max_cost);
}
