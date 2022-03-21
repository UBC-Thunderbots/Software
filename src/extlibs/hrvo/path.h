#pragma once
#include <memory>
#include <optional>
#include <vector>

#include "extlibs/hrvo/path_point.h"
#include "software/geom/vector.h"

class PathPoint;

/**
 * An agent's path
 */
class Path
{
   public:
    /**
     * Default constructor for this path
     */
    explicit Path();

    /**
     * Constructor for this path
     * @param path_points A vector that stores path points, which can be empty
     * @param goal_radius The goal radius for this path
     */
    explicit Path(const std::vector<PathPoint> &path_points, float goal_radius);

    /**
     * Gets the next path point position in this path
     * @return the position of the next path point
     */
    std::optional<Vector> getNextPathPointPosition();

    /**
     * Gets the current path point position in this path
     * @return the position of the current path point
     */
    std::optional<Vector> getCurrentPathPointPosition() const;

    /**
     * Gets the desired speed of the current path point in this path
     * @return the speed at destination for the current path point
     */
    float getDesiredSpeedAtCurrentPathPoint();

    /**
     * Checks if at final path point in a path
     * @return True if the current path point is the final path point in the path,
     * otherwise False
     */
    bool isGoingToFinalPathPoint();

    /**
     * Gets the current path index
     * @return the current path index
     */
    unsigned int getPathIndex() const;

    /**
     * Gets the path point list of this path
     * @return path point vector
     */
    std::vector<PathPoint> getPathVector() const;

    // path radius for this robot. The max distance away from any path point in this path
    float path_radius;


    /**
     * Increments the path point index of this path
     */
    void incrementPathIndex();

    /**
     * Gets the current path point of this path
     * @returns an optional PathPoint
     */
    std::optional<PathPoint> getCurrentPathPoint() const;

    // Assume that the path vector is not empty
    PathPoint getLastPathPoint() const;

   private:
    std::vector<PathPoint> path;
    unsigned int curr_path_index = 0;
};
