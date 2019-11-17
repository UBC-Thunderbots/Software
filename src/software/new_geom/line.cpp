#include "software/new_geom/line.h"

Line::Line() {}

Line::Line(double y_intercept, double slope) : line(Polynomial({y_intercept, slope})) {}

double Line::getYIntercept()
{
    return line.getCoeff(0);
}

double Line::getSlope()
{
    return line.getCoeff(1);
}

void Line::setYIntercept(double y_intercept)
{
    line.setCoeff(0, y_intercept);
}

void Line::setSlope(double slope)
{
    line.setCoeff(1, slope);
}

double Line::valueAt(double val)
{
    return line.valueAt(val);
}
