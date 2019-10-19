#pragma once

#include "software/geom/point.h"
#include <boost/polygon/voronoi.hpp>
#include "software/geom/rectangle.h"
#include "software/geom/circle.h"

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::voronoi_vertex;

/**
 * Finds the points of intersection between the edges of a voronoi diagram and a bounding rectangle
 *
 * @param vd The voronoi diagram
 * @param bounding_box The bounding rectangle
 * @param points The points used to create the Voronoi diagram.
 * @return
 */
std::vector<Point> findVoronoiEdgeRecIntersects(const voronoi_diagram<double>& vd, Rectangle bounding_box, std::vector<Point> points);

/**
 * Find the set of open circles whose origin lies within the given rectangle, and that do not overlap any of the
 * vertices in the Delaunay triangulation of the given voronoi diagram
 *
 * @param vd The voronoi diagram
 * @param bounding_box The bounding rectangle of the Voronoi diagram.
 * @param points The points used to create the Voronoi diagram
 * @return A vector of open circles.
 */
std::vector<Circle> voronoiVerticesToOpenCircles(const voronoi_diagram<double>& vd, Rectangle bounding_box, std::vector<Point> points);
