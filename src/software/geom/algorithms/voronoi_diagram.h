#pragma once

#include <memory>

#include "software/geom/circle.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"

// We pre-declare this boost type so that we can hold a pointer to it as a member, but
// not include the boost header files in this header to prevent them being included in
// other files that include this one, as boost has *many* header files that slow down
// compilation if included in too many places
namespace boost::polygon
{
    template <typename T>
    class voronoi_diagram_traits;
    template <typename T, typename TRAITS>
    class voronoi_diagram;
}  // namespace boost::polygon

class VoronoiDiagram
{
   public:
    explicit VoronoiDiagram(const std::vector<Point> &points);

    VoronoiDiagram() = delete;

    const std::vector<Point> &getPointsUsedForGeneration() const
    {
        return points;
    }

    /**
     * Finds the points of intersection between the edges of a voronoi diagram and a
     * bounding rectangle
     *
     * @param vd The voronoi diagram
     * @param bounding_box The bounding rectangle
     * @return
     */
    std::vector<Point> findVoronoiEdgeRecIntersects(const Rectangle &bounding_box);

    /**
     * Find the set of open circles whose origin lies within the given rectangle, and that
     * do not overlap any of the vertices in the Delaunay triangulation of the given
     * voronoi diagram
     *
     * @param vd The voronoi diagram object
     * @param bounding_box The bounding rectangle of the Voronoi diagram.
     * @return A vector of open circles.
     */
    std::vector<Circle> voronoiVerticesToOpenCircles(const Rectangle &bounding_box);

   private:
    std::shared_ptr<boost::polygon::voronoi_diagram<
        double, boost::polygon::voronoi_diagram_traits<double>>>
        diagram;
    std::vector<Point> points;
};
