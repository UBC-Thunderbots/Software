#pragma once
#include "ai/navigator/path_planner/violation_function.h"
/**
 * A placeholder violation function that always returns zero.
 */
class ZeroViolationFunction : public ViolationFunction
{
   public:
    /**
     * always returns zero.
     * @param point unused parameter
     * @return 0
     */
    double getViolationForPoint(const Point& point) override;
};

double ZeroViolationFunction::getViolationForPoint(const Point& point)
{
    return 0;
}
