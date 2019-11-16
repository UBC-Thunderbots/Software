#pragma once

#include <boost/polygon/voronoi.hpp>

#include "software/geom/circle.h"
#include "software/geom/rectangle.h"
#include "software/new_geom/point.h"

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::voronoi_vertex;

class VoronoiDiagram
{
   private:
    voronoi_diagram<double> diagram;
    std::vector<Point> points;

   public:
    explicit VoronoiDiagram(std::vector<Point> points)
    {
        construct_voronoi(points.begin(), points.end(), &diagram);
        this->points = points;
    }

    VoronoiDiagram() = delete;

    const voronoi_diagram<double> &getDiagram() const
    {
        return diagram;
    }

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
    std::vector<Point> findVoronoiEdgeRecIntersects(Rectangle bounding_box);

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
};
