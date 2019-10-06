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
 * @param rec The bounding rectangle
 * @param points The points used to create the Voronoi diagram.
 * @return
 */
std::vector<Point> findVoronoiEdgeRecIntersects(const voronoi_diagram<double>& vd, Rectangle rec, std::vector<Point> points);

/**
 * From a Voronoi diagram, produce a set of empty circles.
 *
 * @param vd The voronoi diagram
 * @param rec The bounding rectangle of the Voronoi diagram.
 * @param points The points used to create the Voronoi diagram
 * @return A vector of open circles.
 */
std::vector<Circle> voronoiVerticesToOpenCircles(const voronoi_diagram<double>& vd, Rectangle rec, std::vector<Point> points);