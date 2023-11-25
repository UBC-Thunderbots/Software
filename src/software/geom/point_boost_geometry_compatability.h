/**
 * This file contains everything neccesarry to allow our `point` type to be used with
 * boost algorithms. Because it includes parts of boost geometry, it is very expensive
 * in terms of build time, and so we have gone to great lengths to ensure it is not copied
 * into another header file (and thus potentially recursively across the whole codebase)
 */

#ifndef POINT_BOOST_COMPATABILITY_THIS_IS_NOT_IN_A_HEADER
#error This is an extra check to make sure that we are never including this in another  \
     header file. If we do so then all of boost geometry may recursively be included \
     in significant other sections of the codebase, seriously slowing down compilation \
     times. To let this check pass, please confirm that you are *not* including this  \
     in another header file by adding \
     '#define POINT_BOOST_COMPATABILITY_THIS_IS_NOT_IN_A_HEADER' before the \
     inclusion of this file
#endif
#undef POINT_BOOST_COMPATABILITY_THIS_IS_NOT_IN_A_HEADER

#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/polygon/point_concept.hpp>

#include "software/geom/point.h"

// Make our Point class "compatible" with boost. This lets us pass our Points directly
// into boost algorithms
BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(Point, double, cs::cartesian, x, y, setX, setY)
template <>
struct boost::polygon::geometry_concept<Point>
{
    typedef point_concept type;
};
template <>
struct boost::polygon::point_traits<Point>
{
    using coordinate_type = double;

    static coordinate_type get(const Point &point, orientation_2d orient)
    {
        return (orient == HORIZONTAL) ? point.x() : point.y();
    }
};
