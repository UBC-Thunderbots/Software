#include "software/geom/spline2d.h"

SplineSegment2d::SplineSegment2d(double start_val, double end_val,
                                 Polynomial2d polynomial)
    : start_val(start_val), end_val(end_val), polynomial(std::move(polynomial))
{
}

double SplineSegment2d::getParametrizationStartVal() const
{
    return start_val;
}

double SplineSegment2d::getParametrizationEndVal() const
{
    return end_val;
}

const Polynomial2d& SplineSegment2d::getPolynomial() const
{
    return polynomial;
}

SplineSegment2d Spline2d::createSplineSegment2d(double start_val, double end_val,
                                                Polynomial2d polynomial)
{
    return SplineSegment2d(start_val, end_val, std::move(polynomial));
}
