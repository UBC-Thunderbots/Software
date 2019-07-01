#include "ai/world/world.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "geom/rectangle.h"
#include "geom/util.h"
#include "util/parameter/dynamic_parameters.h"

/**
 * The indirect chip and chase target evaluation function returns a target point for the
 * chipper and chaser.
 * Other related evaluation functions are also included in this file.
 */
namespace Evaluation
{
    /**
     * Returns the target point that the chipper and chaser will chip and chase at.
     *
     * Given the enemy robots' positions, filters and creates a vector of
     * triangles to be used as a parameter. The target point is where ball will land
     * according to chipping calibration, as well as where the chaser will meet the ball
     * at.
     *
     * @param world The world in which we want to find the target point
     *
     * @return Point to chip and chase at; If std::nullopt, could not find a good point to
     * chip and chase at
     */
    std::optional<Point> findTargetPointForIndirectChipAndChase(const World& world);

    /**
     * Returns the target point that the chipper and chaser will chip and chase at.
     *
     * Given the vector of triangles without enemy robots, determines if the triangles are
     * empty and if the largest triangle is within reach. If not within reach, scale the
     * target point with the maximum chip power. The target point is where ball will land
     * according to chipping calibration, as well as where the chaser will meet the ball
     * at.
     *
     * @param triangles A vector of triangles with no enemy robots and are within best
     * chip target area
     * @param ball_position Position of the ball
     *
     * @return Point to chip and chase at; If std::nullopt, could not find a good point to
     * chip and chase at
     */
    std::optional<Point> findTargetPointForIndirectChipAndChase(
        const std::vector<LegacyTriangle>& triangles, Point ball_position);

    /**
     * Returns a vector of all possible triangles between enemy players and rectangular
     * chip area.
     *
     * Creates a vector of triangles that picks all permutations of points from the
     * list of all non-goalie enemy players as well as the four points returned by
     * get_chip_area_target_corners.
     *
     * @param world The world in which we want to find the target point
     * @param enemy_players Vector of enemy robots' positions
     *
     * @return Vector of all possible triangles between enemy players and chip area
     */
    std::vector<LegacyTriangle> getAllTrianglesBetweenEnemyPlayers(
        const World& world, std::vector<Point> enemy_players);

    /**
     * Returns a vector of triangles that only contains triangles without enemy robots.
     *
     * Given a vector of triangles, shrink each triangle slightly (to avoid counting the
     * robot's width within the triangle), filters out the triangles that consist of enemy
     * robots and returns a new vector that only contains triangles without enemy robots.
     *
     * @param triangles Vector of triangles
     * @param enemy_players Vector of enemy robots' positions
     *
     * @return Vector of triangles with no enemy robots
     */
    std::vector<LegacyTriangle> findOpenTriangles(std::vector<LegacyTriangle> triangles,
                                                  std::vector<Point> enemy_players);

    /**
     * Returns the center point the given triangle.
     *
     * @param triangle
     *
     * @return Centre point of triangle
     */
    Point getTriangleCenter(LegacyTriangle triangle);

    /**
     * Returns the area of the given triangle.
     *
     * @param triangle
     *
     * @return Area of triangle in meters squared
     */
    double getTriangleArea(LegacyTriangle triangle);

    /**
     * Remove all Triangles in a given list whose centers do not fall
     * within the best, rectangular chip target area.
     *
     * Given a vector of triangles and rectangular target area, compares center point of
     * each triangle and positions of each side of rectangle. If center point of triangle
     * falls within rectangle, it is placed in a vector of valid triangles.
     *
     * @param rectangle Chip target area
     * @param triangles Vector of triangles
     *
     * @return Valid triangles that are within the chip target area
     */
    std::vector<LegacyTriangle> removeTrianglesOutsideRectangle(
        Rectangle rectangle, std::vector<LegacyTriangle> triangles);

    /**
     * Returns a Rectangle of best target area to chip and chase at.
     *
     * This rectangle is from ball's position to enemy's end of field. All points are
     * 'inset' distance away from each edge of the field to allow a buffer for catching
     * and prevent ball from leaving the field.
     *
     * @param world The world in which we want to find the target point
     * @param inset Distance away from each edge of field in meters
     *
     * @return Rectangle of best target area
     */
    Rectangle findBestChipTargetArea(const World& world, double inset);

    /**
     * Returns the largest triangle that meets the area and edge length thresholds.
     *
     * Given a vector of triangles, returns the largest triangle with area greater than
     * minimum area a chip target triangle must have to be considered valid, and all edge
     * lengths greater than minimum edge length for the triangle to be considered valid.
     *
     * @param allTriangles Vector of triangles
     * @param min_area Minimum area that a chip target triangle must have to be considered
     * valid in meters squared
     * @param min_edge_len Minimum length in meters that each edge of the chip target
     * triangle must have for the triangle to be considered valid
     * @param min_edge_angle Minimum angle in degrees that any two edges of a chip
     * target triangle must have for that triangle to be considered valid
     *
     * @return Largest triangle
     */
    std::optional<LegacyTriangle> getLargestValidTriangle(
        std::vector<LegacyTriangle> allTriangles, double min_area = 0,
        double min_edge_len = 0, double min_edge_angle = 0);
};  // namespace Evaluation
