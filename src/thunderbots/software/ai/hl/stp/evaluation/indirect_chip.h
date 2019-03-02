#include "ai/world/world.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "util/parameter/dynamic_parameters.h"

/**
 * The indirect chip and chase target evaluation function returns a target point for the
 * chipper and chaser.
 * Other related evaluation functions are also included in this file.
 */
namespace Evaluation
{
    typedef Point Vector2;
    template <size_t N>
    using Poly     = std::array<Vector2, N>;
    using Triangle = Poly<3>;

    /**
     * Returns the target point that the chipper will shoot at and the chaser will meet
     * ball at. Given the enemy team robot's positions, filters and creates a vector of
     * triangles to be used as a parameter. The target is where ball will land according
     * to chipping calibration.
     *
     * @param World Object
     *
     * @return Point to chip and chase at; If empty, point is not valid
     */
    std::optional<Point> indirect_chip_and_chase_target(const World& world);

    /**
     * Returns the target point that the chipper will shoot at and the chaser will meet
     * ball at. Given the filtered vector of triangles, determines if the triangles are
     * empty and if the largest triangle is within reach. The target is where ball will
     * land according to chipping calibration.
     *
     * @param triangles A vector of triangles that is already filtered
     * @param ball_position Position of the ball
     *
     * @return Point to chip and chase at; If empty, point is not valid
     */
    std::optional<Point> indirect_chip_and_chase_target(
        const std::vector<Triangle>& triangles, Point ball_position);

    /**
     * Creates a vector of triangles that picks all permutations of points from the
     * list of all non-goalie enemy players as well as the four points returned by
     * get_chip_area_target_corners.
     *
     * @param World Object
     * @param enemy_players
     *
     * @return Vector of triangles
     */
    std::vector<Triangle> get_all_triangles(const World& world,
                                            std::vector<Point> enemy_players);

    /**
     * Given a vector of triangles, returns a new vector that only contains triangles
     * without enemy robots.
     *
     * @param triangles
     * @param enemy_players
     *
     * @return Vector of triangles with no enemy robots
     */
    std::vector<Triangle> filter_open_triangles(std::vector<Triangle> triangles,
                                                std::vector<Point> enemy_players);

    /**
     * Returns the center point and area of the given triangle.
     *
     * @param triangle
     *
     * @return Centre point of triangle
     * @return Area of triangle
     */
    std::pair<Point, double> get_triangle_center_and_area(Triangle triangle);

    /**
     * Remove all Triangles in a given list whose centers do not fall
     * within the rectangle returned by get_chip_target_area.
     *
     * @param World Object
     * @param triangles
     *
     * @return Valid triangles
     */
    std::vector<Triangle> remove_outofbounds_triangles(const World& world,
                                                       std::vector<Triangle> triangles);

    /**
     * Returns four Points representing a rectangular region to chip and
     * chase. This rectangle is from ball's position to enemy's end of field.
     * All points are 'inset' distance away from each edge of the field to allow a
     * buffer for catching and prevent ball from leaving the field.
     *
     * @param World Object
     * @param inset Distance away from each edge of field
     *
     * @return Four points for rectangle
     */
    std::vector<Point> get_chip_target_area_corners(const World& world, double inset);

    /**
     * Given a vector of triangles, returns the largest triangles with area greater
     * than min_area and all edge lengths greater than min_edge_len.
     *
     * @param allTriangles Vector of triangles
     * @param min_area = 0
     * @param min_edge_len = 0
     * @param min_edge_angle = 0
     *
     * @return Largest triangle
     * @return valid Largest triangle is found
     */
    std::pair<Triangle, bool> get_largest_triangle(std::vector<Triangle> allTriangles,
                                                   double min_area       = 0,
                                                   double min_edge_len   = 0,
                                                   double min_edge_angle = 0);

};  // namespace Evaluation
