#pragma once

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/new_geom/circle.h"

/**
 * An Obstacle represented by a Circle
 */
class CircleObstacle : public Obstacle
{
   public:
    CircleObstacle() = delete;

    /**
     * Construct a circle obstacle with a Circle
     *
     * @param circle Circle to make obstacle with
     */
    explicit CircleObstacle(const Circle circle);

    bool contains(const Point& p) const override;
    double distance(const Point& p) const override;
    bool intersects(const Segment& segment) const override;
    std::string toString(void) const override;

    /**
     * Gets the underlying circle
     *
     * @return circle
     */
    const Circle getCircle(void) const;

   private:
    Circle circle_;
};

/**
 * Implements the << operator for printing
 *
 * @param ostream The stream to print to
 * @param circle_obstacle The CircleObstacle to print
 *
 * @return The output stream with the string representation of the class appended
 */
std::ostream& operator<<(std::ostream& os, const CircleObstacle& circle_obstacle);
