#include "software/geom/util.h"
#include "software/new_geom/ray.h"
#include "software/new_geom/rectangle.h"
#include "software/new_geom/segment.h"

/**
 * Computes the intersection of two segments.
 *
 * @param first
 * @param second
 *
 * @return  one of:
 *          - an empty vector if no intersections
 *          - a vector containing a single point of intersection
 *          - a vector containing two points representing the line segment of the overlap
 * if both segments are collinear and overlapping
 */
std::vector<Point> intersection(const Segment &first, const Segment &second);

/**
 * Computes the points of intersection between a rectangle and a line segment.
 * Will contain duplicates if segment goes through rectangle corners.
 *
 * @param rectangle
 * @param segment
 *
 * @return a vector containing the points of intersection between a rectangle and a
 * segment
 */
std::vector<Point> intersection(const Rectangle &rectangle, const Segment &segment);

/**
 * Computes the points of intersection between a ray and a line segment.
 *
 * @param ray
 * @param segment
 *
 * @return  one of:
 *          - an empty vector if no intersections
 *          - a vector containing a single point of intersection
 *          - a vector containing two points representing the line segment of the overlap
 * if the ray and segment are collinear and overlapping
 */
std::vector<Point> intersection(const Ray &ray, const Segment &segment);

/**
 * Computes the point of intersection between two lines.
 * Note: this computes the intersection of two lines, not line segments.
 *
 * @pre the lines are not parallel
 *
 * @params a, b points that represent the first line
 * @params c, d points that represent the second line
 *
 * @return the point of intersection, if it exists
 */
std::optional<Point> intersection(const Point &a, const Point &b, const Point &c,
                                  const Point &d);

/**
 * Computes the points of intersection between a rectangle and a ray.
 * Will contain duplicates if ray goes through rectangle corners.
 *
 * @param rectangle
 * @param ray
 *
 * @return a vector containing the points of intersection between a rectangle and a ray
 */
std::vector<Point> intersection(const Rectangle &rectangle, const Ray &ray);

/**
 * Computes the point of intersection between two rays.
 * Note: Returns nullopt if rays are overlapping and parallel.
 *
 * @param first
 * @param second
 *
 * @return the point of intersection, if it exists
 */
std::optional<Point> intersection(const Ray &first, const Ray &second);
