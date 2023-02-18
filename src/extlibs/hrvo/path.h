#pragma once
#include <memory>
#include <optional>
#include <vector>

#include "extlibs/hrvo/path_point.h"
#include "software/geom/vector.h"

/**
 * An agent's path
 */
class AgentPath
{
public:
    /**
     * Default constructor for this path
     */
    explicit AgentPath();

    /**
     * Constructor for this path
     * @param path_points A vector that stores path points, which can be empty
     * @param goal_radius The goal radius for this path
     */
    explicit AgentPath(const std::vector<PathPoint> &path_points, float goal_radius);

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

    /**
     * Gets the path radius for this agent
     * @returns path radius for this agent
     */
    float getPathRadius() const;

private:
    std::vector<PathPoint> path;
    unsigned int curr_path_index = 0;
    // path radius for this robot. The max distance away from any path point in this path
    float path_radius;
};
