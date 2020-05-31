#pragma once

#include "software/new_geom/ray.h"
#include "software/new_geom/spline2d.h"

// TODO: class-wide jdoc, should roughly explain math, guarantees, etc.? Or maybe links
//       below are enough
// TODO: link to https://www.ibiblio.org/e-notes/Splines/Intro.htm
// TODO: link to https://www.ibiblio.org/e-notes/Splines/b-int.html
class InterpolatingCubicBSpline : public Spline2d
{
   public:
    InterpolatingCubicBSpline() = delete;

    // TODO: better names then `start` and `end`
    // TODO: finish this jdoc. Referencing a diagram here would probably really help,
    //       maybe ascii??
    /**
     * Create a Cubic B-spline that:
     *  - starts from the start point of `start` and is tangent to it
     *  - end from the start point of `end` and is tangent to it
     * @param start
     * @param end
     * @param intermediate_knots The intermediate points between `start` and
     *                                    `end` that this spline will interpolate
     */
    InterpolatingCubicBSpline(Ray start, Ray end,
                              std::vector<Point> intermediate_knots);

    const Point getValueAt(double val) const override;

    const std::vector<Point> getKnots() const override;

    // TODO: jdoc for this
    const std::vector<Point> getControlPoints() const;

    size_t getNumKnots() const override;

    const Point getStartPoint() const override;

    const Point getEndPoint() const override;

    const std::vector<SplineSegment2d> getSplineSegments() const override;

};