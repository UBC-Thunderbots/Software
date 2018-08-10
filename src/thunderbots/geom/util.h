#pragma once

#include <cstddef>
#include <vector>
#include "geom/point.h"
#include "geom/shapes.h"

template <size_t N>
using Poly     = std::array<Vector2, N>;
using Triangle = Poly<3>;
using Quad     = Poly<4>;

constexpr double EPS = 1e-9;

constexpr double EPS2 = EPS * EPS;

constexpr int sign(double n)
{
    return n > EPS ? 1 : (n < -EPS ? -1 : 0);
}

inline Triangle triangle(const Point &a, const Point &b, const Point &c)
{
    return {a, b, c};
}
inline Quad quad(const Point &a, const Point &b, const Point &c, const Point &d)
{
    return {a, b, c, d};
}

/**
 * Signed magnitude of the projection of `second` on `first`
 */
double proj_len(const Vector2 &first, const Vector2 &second);

/**
 * Signed magnitude of the projection of `first.start -> second` on `first`
 */
double proj_len(const Seg &first, const Vector2 &second);

/*
 * The family of `contains` functions determins whether
 * the second parameter is contained, even if partially,
 * inside the first parameter.
 */

bool contains(const Triangle &out, const Vector2 &in);
bool contains(const Circle &out, const Vector2 &in);
bool contains(const Circle &out, const Seg &in);
bool contains(const Ray &out, const Vector2 &in);
bool contains(const Seg &out, const Vector2 &in);
bool contains(const Rect &out, const Vector2 &in);

/*
 * The family of `intersects` functions determines whether there
 * exists an intersection between one object and another.
 */

bool intersects(const Triangle &first, const Circle &second);
bool intersects(const Circle &first, const Triangle &second);
bool intersects(const Circle &first, const Circle &second);
bool intersects(const Seg &first, const Circle &second);
bool intersects(const Circle &first, const Seg &second);
bool intersects(const Seg &first, const Seg &second);
bool intersects(const Ray &first, const Seg &second);
bool intersects(const Seg &first, const Ray &second);

/*
 * The family of `dist` functions calculates the unsigned distance
 * between one object and another.
 */
double dist(const Vector2 &first, const Vector2 &second);
double dist(const Seg &first, const Seg &second);

double dist(const Vector2 &first, const Seg &second);
double dist(const Seg &first, const Vector2 &second);

double dist(const Line &first, const Vector2 &second);
double dist(const Vector2 &first, const Line &second);

double distsq(const Vector2 &first, const Seg &second);
double distsq(const Seg &first, const Vector2 &second);
double distsq(const Vector2 &first, const Vector2 &second);

bool is_degenerate(const Seg &seg);
bool is_degenerate(const Ray &seg);
bool is_degenerate(const Line &line);

double len(const Seg &seg);
double len(const Line &line);

double lensq(const Seg &seg);
double lensq(const Line &line);

template <size_t N>
Vector2 get_vertex(const Poly<N> &poly, unsigned int i);
template <size_t N>
void set_vertex(Poly<N> &poly, unsigned int i, Vector2 &v);

/**
 * Gets the side that is connected to the vertex with index provided
 * and also connected to the vertex with the next index.
 */
template <size_t N>
Seg get_side(const Poly<N> &poly, unsigned int i);

/**
 * Checks if 3 points are collinear.
 *
 * \param[in] a a point
 *
 * \param[in] b a point
 *
 * \param[in] c a point
 *
 * \returns true if any two points are within EPS distance to each other. (If
 * any two of three points are within EPS distance of each other, they are
 * essentially the same point and the two points will form the same line.)
 *
 * \returns true if the cross product of the two lines formed by the three
 * points are smaller than EPS
 */
bool collinear(const Point &a, const Point &b, const Point &c);

/**
 * Performs an angle sweep.
 * Suppose in this world, all objects are circles of fixed radius.
 * You are at point \p src, and you want to shoot a ray between \p p1 and \p p2.
 * This function calculates the largest open angle interval that you can shoot.
 *
 * \pre \p p1 must be to the right of \p p2.
 * In other words, if there is a counterclockwise ordering, \p p1 is before \p
 * p2 from \p src's point of view.
 *
 * \pre The angle \p p1, \p src, \p p2 must not be greater than 180 degrees.
 *
 * \pre \p src must not be between \p p1 and \p p2.
 *
 * \param[in] src the location where you are standing.
 *
 * \param[in] p1 the location of the right-hand edge of the target area.
 *
 * \param[in] p2 the location of the left-hand edge of the target area.
 *
 * \param[in] obstacles the coordinates of the centres of the obstacles.
 *
 * \param[in] radius the radii of the obstacles.
 *
 * \return the best direction to shoot and the size of the angle centered around
 * that direction that is completely free of obstacles,
 * or <code>(<var>p</var>, 0)</code> for some unspecified <var>p</var> if there
 * is no free path.
 */
std::pair<Point, Angle> angle_sweep_circles(
    const Point &src, const Point &p1, const Point &p2,
    const std::vector<Point> &obstacles, const double &radius);

/**
 * Gets all angle.
 *
 * \pre The points \p p1,\p p2, and \p src has to not be collinear and \p src
 * can't be within the radius of the obstacle
 *
 * \param[in] src the location where you are standing.
 *
 * \param[in] p1 the location of the right-hand edge of the target area.
 *
 * \param[in] p2 the location of the left-hand edge of the target area.
 *
 * \param[in] obstacles the coordinates of the centres of the obstacles.
 *
 * \param[in] radius the radii of the obstacles.
 *
 * \returns a vector of all possible pairs of directions and angles to a target
 * area. An empty vector is returned if the preconditions aren't satisfied.
 */
std::vector<std::pair<Point, Angle>> angle_sweep_circles_all(
    const Point &src, const Point &p1, const Point &p2,
    const std::vector<Point> &obstacles, const double &radius);

/**
 * returns a list of points that lie exactle "buffer" distance awaw from the
 * line seg
 */
std::vector<Point> seg_buffer_boundaries(
    const Point &a, const Point &b, double buffer, int num_points);

/**
 * returns a list of points that lie on the border of the circle
 */
std::vector<Point> circle_boundaries(
    const Point &centre, double radius, int num_points);

/**
 * Finds the Point on line segment closest to point.
 *
 * \param[in] centre the point.
 *
 * \param[in] segA one end of the line segment.
 *
 * \param[in] segB the other end of the line segment.
 *
 * \return the Point on line segment closest to centre point.
 */
Point closest_lineseg_point(
    const Point &p, const Point &segA, const Point &segB);

/**
 * Finds the points of intersection between a circle and a line.
 * There may be zero, one, or two such points.
 *
 * \param[in] centre the centre of the circle.
 *
 * \param[in] radius the radius of the circle.
 *
 * \param[in] segA one point on the line.
 *
 * \param[in] segB another point on the line.
 *
 * \return the points of intersection.
 */
std::vector<Point> line_circle_intersect(
    const Point &centre, double radius, const Point &segA, const Point &segB);

/**
 * Finds the points of intersection between a circle and a line.
 * There may be zero, one, or two such points.
 *
 * \param[in] r the rectangle.
 *
 * \param[in] segA one point on the line.
 *
 * \param[in] segB another point on the line.
 *
 * \return the points of intersection.
 */
std::vector<Point> line_rect_intersect(
    const Rect &r, const Point &segA, const Point &segB);

/**
 * Find where a vector intersect a rectangle
 * return the center of the rectangle if there is no intersection
 *
 * \param[in] r the rectangle.
 *
 * \param[in] segA one point on the line.
 *
 * \param[in] segB another point on the line.
 *
 * \return the points of intersection.
 */
Point vector_rect_intersect(
    const Rect &r, const Point &segA, const Point &segB);

/**
 * Clips a point to a rectangle boundary.
 *
 * \param[in] p the point to clip.
 *
 * \param[in] bound1 one corner of the rectangle.
 *
 * \param[in] bound2 the diagonally-opposite corner of the rectangle.
 *
 * \return the closest point to \p p that lies within the rectangle.
 */
Point clip_point(const Point &p, const Point &bound1, const Point &bound2);

/**
 * Clips a point to a rectangle boundary.
 *
 * \param[in] p the point to clip.
 *
 * \param[in] r the rectangle.
 *
 * \return the closest point to \p p that lies within the rectangle.
 */
Point clip_point(const Point &p, const Rect &r);

/**
 * Computes whether there is a unique intersection of two lines.
 *
 * \param[in] a a point on the first line.
 *
 * \param[in] b another point on the first line.
 *
 * \param[in] c a point on the second line.
 *
 * \param[in] d another point on the second line.
 *
 * \return whether there is one and only one answer
 */
bool unique_line_intersect(
    const Point &a, const Point &b, const Point &c, const Point &d);

/**
 * Computes the intersection of two lines.
 *
 * \pre The lines must be non-parallel.
 *
 * \param[in] a a point on the first line.
 *
 * \param[in] b another point on the first line.
 *
 * \param[in] c a point on the second line.
 *
 * \param[in] d another point on the second line.
 *
 * \return the point of intersection.
 */
Point line_intersect(
    const Point &a, const Point &b, const Point &c, const Point &d);

std::vector<Point> line_intersect(const Seg &a, const Seg &b);

/**
 * Reflects a ray incident on origin given the normal of the reflecting plane.
 *
 * \pre the normal vector cannot have length smaller than EPS.
 *
 * \param[in] v the incident ray to reflect.
 *
 * \param[in] n the vector normal to the reflecting plane.
 *
 * \return the reflected ray.
 */
Point reflect(const Point &v, const Point &n);

/**
 * Reflects a point across a line.
 *
 * \param[in] a a point on the line.
 *
 * \param[in] b another point on the line.
 *
 * \param[in] p the point to reflect.
 *
 * \return the reflection of \p p across the line.
 */
Point reflect(const Point &a, const Point &b, const Point &p);

/**
 * Given a cone shooting from the origin, determines the furthest location from
 * the origin, at which to place a circle to block the cone.
 *
 * \pre The cone must have nonzero area.
 *
 * \pre \p b must be counterclockwise of \p a.
 *
 * \param[in] a the starting angle of the cone.
 *
 * \param[in] b the ending angle of the cone.
 *
 * \param[in] radius the radius of the circle with which to block the cone.
 *
 * \return the blocking position.
 */
Point calc_block_cone(const Point &a, const Point &b, const double &radius);

/**
 * Given a cone shooting from a point P, determines the furthest location from
 * P, at which to place a circle to block the cone.
 *
 * \pre The cone must have nonzero area.
 *
 * \pre \p b must be counterclockwise of \p a.
 *
 * \param[in] a the starting angle of the cone.
 *
 * \param[in] b the ending angle of the cone.
 *
 * \param[in] radius the radius of the circle with which to block the cone.
 *
 * \param[in] p the source of the cone.
 *
 * \return the blocking position.
 */
Point calc_block_cone(
    const Point &a, const Point &b, const Point &p, const double &radius);

/**
 * Used for defender_blocks_goal
 * a = goal post side
 * c = ball position
 * g = goalie position
 * returns the other ray/cone boundary that is not blocked by goalie
 * I.e. if p is return value,
 * then points to the other side of line p-c is not covered by goalie.
 */
Point calc_block_other_ray(const Point &a, const Point &c, const Point &g);

/**
 * Ported code
 * Checks whether the goalie is in the middle of both goal posts.
 *
 * \param[in] a goal post position
 *
 * \param[in] b other goal post position
 *
 * \param[in] c ball position
 *
 * \param[in] g goalie position
 */
bool goalie_block_goal_post(
    const Point &a, const Point &b, const Point &c, const Point &g);

/**
 * Calculates a defender position to block the ball.
 *
 * \pre the goalie is between the two goal posts, as seen from the ball.
 *
 * \param[in] a right goal post position, from goalie's perspective
 *
 * \param[in] b other goal post position
 *
 * \param[in] c ball position
 *
 * \param[in] g goalie position
 *
 * \param[in] r radius of defending robot
 */
Point calc_block_cone_defender(
    const Point &a, const Point &b, const Point &c, const Point &g,
    const double &r);

/*
 * Ported code from CM geom util
 */

/**
 * returns perpendicular offset from line x0-x1 to point p
 */
double offset_to_line(Point x0, Point x1, Point p);

/**
 * returns perpendicular offset from line x0-x1 to point p
 */
double offset_along_line(Point x0, Point x1, Point p);

/**
 * returns nearest point on segment a0-a1 to line b0-b1
 */
Point segment_near_line(Point a0, Point a1, Point b0, Point b1);

/**
 * intersection of two segments?
 */
Point intersection(Point a1, Point a2, Point b1, Point b2);

/**
 * gives counterclockwise angle from <a-b> to <c-b>
 */
Angle vertex_angle(Point a, Point b, Point c);

/**
 * returns time of closest point of approach of two points
 * moving along constant velocity vectors.
 */
double closest_point_time(Point x1, Point v1, Point x2, Point v2);

/**
 * found out if a point is in the vector's direction or against it
 * if the point normal to the vector, return false
 *
 * param[in] offset is the position of the origin of the vector
 *
 * param[in] dir is the direction of the vector
 *
 * param[in] p is the point is question
 */
bool point_in_front_vector(Point offset, Point dir, Point p);

/**
 * Returns true if v2 is clockwise relative to v1
 */
#warning this should work but hasn't been properly tested
bool is_clockwise(Point v1, Point v2);

/**
 * Returns the circle's tangent points.
 *
 * Returns the points on the circle that form tangent lines with the start point
 */
std::pair<Point, Point> get_circle_tangent_points(
    const Point &start, const Circle &circle, double buffer = 0.0);

bool point_is_to_right_of_line(const Seg &line, const Point &point);

/**
 * Returns the mean of a list of points
 *
 * @param points the vector of points
 * @return the mean point of points
 */
Point getPointsMean(const std::vector<Point> &points);

/**
 * Returns the variance of a list of Points
 *
 * @param points the vector of points
 * @return the variance of the list of points
 */
double getPointsVariance(const std::vector<Point> &points);
