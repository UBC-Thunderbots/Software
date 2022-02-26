#pragma once
#include <vector>
#include <memory>
#include "extlibs/hrvo/vector2.h"
#include "extlibs/hrvo/path_point.h"

class PathPoint;

/**
 * An agent's path
 */
class Path {
    
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
        Vector2 getNextPathPointPosition();

        /**
         * Gets the current path point position in this path
         * @return the position of the current path point
         */
        Vector2 getCurrentPathPointPosition();

        /**
         * Gets the desired speed of the current path point in this path
         * @return the speed at destination for the current path point
         */
        float getDesiredSpeedAtCurrentPathPoint();

        /**
         * Checks if at final path point in a path
         * @return True if the current path point is the final path point in the path, otherwise False
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

        // goal radius for this robot
        float goal_radius;


    private:
        std::vector<PathPoint> path;
        unsigned int curr_goal_index = 0;

};