#pragma once

#include <memory>
#include <sstream>
#include <vector>

#include "proto/primitive.pb.h"
#include "proto/visualization.pb.h"
#include "software/ai/navigator/obstacle/obstacle_visitor.h"
#include "software/geom/algorithms/axis_aligned_bounding_box.h"
#include "software/geom/algorithms/signed_distance.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"

/**
 * An obstacle is an area to avoid for navigation
 */
class Obstacle
{
   public:
    Obstacle()          = default;
    virtual ~Obstacle() = default;

    /**
     * Determines whether the given Point is contained within this Obstacle
     *
     * @param p Point to check if contained by obstacle
     * @param t_sec Time in seconds into the future to check if point is contained.
     *
     * @return whether the Point p is contained within this Obstacle
     */
    virtual bool contains(const Point& p, const double t_sec = 0) const = 0;

    /**
     * Gets the minimum distance from the obstacle to the point
     *
     * @param p Point to get distance to
     * @param t_sec Time in seconds into the future to get distance to.
     *
     * @return distance to point
     */
    virtual double distance(const Point& p, const double t_sec = 0) const = 0;

    /**
     * Gets the signed distance from the obstacle's perimeter to the point. That is, if
     * point is inside the obstacle then distance will be negative. See
     * https://iquilezles.org/articles/distfunctions2d/ for details on the maths
     *
     * @param point Point to get distance to
     * @param t_sec Time in seconds into the future to get distance to.
     * @return distance from point to nearest point on perimeter of obstacle. Positive if
     * outside, negative if inside
     */
    virtual double signedDistance(const Point& point, const double t_sec = 0) const = 0;

    /**
     * Determines whether the given Segment intersects this Obstacle
     *
     * @param segment Segment to check if intersects with this Obstacle
     * @param t_sec Time in seconds into the future to check if segment intersects.
     *
     * @return true if the given Segment intersects this Obstacle
     */
    virtual bool intersects(const Segment& segment, const double t_sec = 0) const = 0;

    /**
     * Finds the Point closest to the given Point that is outside of the obstacle.
     *
     * @param p the point.
     *
     * @return the Point on polygon closest to point.
     */
    virtual Point closestPoint(const Point& p) const = 0;

    /**
     * Determines what coordinates on the field are blocked by this Obstacle
     *
     * @param resolution_size The minimum size of the grid cells to rasterize the obstacle
     * into
     *
     * @return a set of points covered by this Obstacle
     */
    virtual std::vector<Point> rasterize(const double resolution_size) const = 0;

    /**
     * Creates an obstacle proto representation
     *
     * @return the obstacle proto
     */
    virtual TbotsProto::Obstacle createObstacleProto() const = 0;

    /**
     * Create the axis aligned bounding box for this obstacle
     *
     * @param inflation_radius The radius to inflate the obstacle by before creating the
     * AABB
     *
     * @return Rectangle representing the axis aligned bounding box
     */
    virtual Rectangle axisAlignedBoundingBox(const double inflation_radius) const = 0;

    /**
     * Output string to describe the obstacle
     *
     * @return string that describes the obstacle
     */
    virtual std::string toString(void) const = 0;

    /**
     * Accepts an Obstacle Visitor and calls the visit function
     *
     * @param visitor An Obstacle Visitor
     */
    virtual void accept(ObstacleVisitor& visitor) const = 0;
};

/**
 * Creates an obstacle proto from polygon or circle
 *
 * @param the geom object
 * @return the obstacle proto
 */
TbotsProto::Obstacle createObstacleProto(const Polygon& polygon);
TbotsProto::Obstacle createObstacleProto(const Rectangle& rectangle);
TbotsProto::Obstacle createObstacleProto(const Circle& circle);
TbotsProto::Obstacle createObstacleProto(const Stadium& stadium);

/**
 * We use a pointer to Obstacle to support inheritance
 * Note: this is a convenience typedef
 */
using ObstaclePtr = std::shared_ptr<Obstacle>;

/**
 * Implements the << operator for printing
 *
 * @param os The ostream to print to
 * @param obstacle_ptr The ObstaclePtr to print
 *
 * @return The output stream with the string representation of the class appended
 */
inline std::ostream& operator<<(std::ostream& os, const ObstaclePtr& obstacle_ptr)
{
    os << obstacle_ptr->toString();
    return os;
}
