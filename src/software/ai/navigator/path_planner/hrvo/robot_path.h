#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "software/ai/navigator/path_planner/hrvo/path_point.h"
#include "software/geom/vector.h"

/**
 * A robot's path
 */
class RobotPath
{
   public:
    /**
     * Default constructor for this path
     */
    explicit RobotPath();

    /**
     * Constructor for this path
     * @param path_points A vector that stores path points, which can be empty
     * @param goal_radius The goal radius for this path
     */
    explicit RobotPath(const std::vector<PathPoint> &path_points, double goal_radius);

    /**
     * Checks if at final path point in a path
     * @return True if the current path point is the final path point in the path,
     * otherwise False
     */
    bool isGoingToFinalPathPoint();

    /**
     * Increments the path point index of this path
     */
    void incrementPathIndex();

    /**
     * Gets the current path point of this path
     * @returns an optional PathPoint
     */
    std::optional<PathPoint> getCurrentPathPoint() const;

    // TODO: Added for hacky integration. Remove!
    std::optional<PathPoint> getFinalPathPoint() const;

    /**
     * Gets the path radius for this agent
     * @returns path radius for this agent
     */
    double getPathRadius() const;

   private:
    std::vector<PathPoint> path;
    unsigned int curr_path_index = 0;
    // path radius for this robot. The max distance away from any path point in this path
    double path_radius;
};
