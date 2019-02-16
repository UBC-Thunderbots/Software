#include "ai/world/world.h"
#include "geom/point.h"

namespace Evaluation
{
    typedef Point Vector2;
    template <size_t N>
    using Poly     = std::array<Vector2, N>;
    using Triangle = Poly<3>;

    // Minimum area of chip target triangle
    const double MIN_CHIP_TRI_AREA(0.5);
    // Minimum edge length of chip target triangle
    const double MIN_CHIP_TRI_EDGE_LEN(0.8);
    // Percentage of distance to center of triangle to return as target
    const double CHIP_CHERRY_POWER_DOWNSCALE(0.85);
    // Maximum power the robot can chip the ball at without malfunctions
    const double MAX_CHIP_POWER(8.0);
    // Closest distance to edge of field that the robot could chip and chase to
    const double CHIP_TARGET_AREA_INSET(0.3);

    /**
     * Returns the target point that the chipper will shoot at and the chaser will meet
     * ball at. The target is where ball will land according to chipping calibration.
     *
     * @param World Object
     *
     * @return Target point to chip and chase at
     */
    std::pair<Point, bool> indirect_chip_and_chase_target(const World& world);

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
     * @return Valid triangless
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
     * @return true Largest triangle is found
     */
    std::pair<Triangle, bool> get_largest_triangle(std::vector<Triangle> allTriangles,
                                                   double min_area       = 0,
                                                   double min_edge_len   = 0,
                                                   double min_edge_angle = 0);

};  // namespace Evaluation
