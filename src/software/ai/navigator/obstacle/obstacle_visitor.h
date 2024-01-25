#pragma once

#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/geom/circle.h"
#include "software/geom/polygon.h"
#include "software/geom/rectangle.h"

// We forward-declare GeomObstacle because if we include them we induce a
// circular dependency between the Individual library for each obstacle and this
// visitor.
template <typename GEOM_TYPE>
class GeomObstacle;

/**
 * This class provides an interface for all Obstacle Visitors. The Visitor design pattern
 * allows us to perform operations on Obstacle objects without needing to check which
 * concrete type it is with an if/else statement, and we don't need to pollute the
 * Obstacle classes with information or functions that are specific to the task we
 * want to perform.
 */
class ObstacleVisitor
{
   public:
    ObstacleVisitor()          = default;
    virtual ~ObstacleVisitor() = default;

    /**
     * Visits an Obstacle to perform an operation.
     *
     * @param The Obstacle to visit
     */
    virtual void visit(const GeomObstacle<Circle> &geom_obstacle)    = 0;
    virtual void visit(const GeomObstacle<Polygon> &geom_obstacle)   = 0;
    virtual void visit(const GeomObstacle<Rectangle> &geom_obstacle) = 0;
    virtual void visit(const GeomObstacle<Stadium> &geom_obstacle)   = 0;
};
