#pragma once

#include "software/new_geom/ray.h"
#include "software/new_geom/spline2d.h"

// TODO: class-wide jdoc, should roughly explain math, guarantees, etc.? Or maybe links
//       below are enough
// TODO: link to https://www.ibiblio.org/e-notes/Splines/Intro.htm
// TODO: link to https://www.ibiblio.org/e-notes/Splines/b-int.html
class CubicBezierSpline2d : public Spline2d
{
   public:
    CubicBezierSpline2d() = delete;

    // TODO: better names then `start` and `end`
    // TODO: finish this jdoc. Referencing a diagram here would probably really help,
    //       maybe ascii??
    // TODO: need to make sure you're explicit about the directions of the start and
    //       end vectors (ie. they're symmetric)
    /**
     * Create a CubicBezierSpline2d that:
     *  - starts from the start point of `start` and is tangent to it
     *  - end from the start point of `end` and is tangent to it
     * @param start
     * @param end
     * @param intermediate_knots The intermediate points between `start` and
     *                                    `end` that this spline will interpolate
     */
    CubicBezierSpline2d(const Point& start_point, const Vector& start_vector,
                        const Point& end_point, const Vector& end_vector,
                        std::vector<Point> intermediate_knots);

    const Point getValueAt(double t) const override;

    const std::vector<Point> getKnots() const override;

    // TODO: jdoc for this
    const std::vector<Point>& getControlPoints() const;

    size_t getNumKnots() const override;

    /**
     * Get the number of segments on this spline.
     * @return The number of segments on this spline.
     */
    size_t getNumSegments() const;

    std::vector<double> getKnotVector() const override;

    const Point getStartPoint() const override;

    const Point getEndPoint() const override;

    const std::vector<SplineSegment2d> getSplineSegments() const override;

   private:
    // TODO: jdoc
    std::vector<Point> computeControlPoints(const Point& start_point,
                                            const Vector& start_vector,
                                            const Point& end_point,
                                            const Vector& end_vector,
                                            std::vector<Point> intermediate_knots);

    std::vector<Point> control_points;
};