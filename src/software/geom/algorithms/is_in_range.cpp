#include "is_in_range.h"

bool isInRange(const double val, const double range1, const double range2)
{
    if (range1 <= range2)
    {
        return val >= range1 && val <= range2;
    }
    else
    {
        return val >= range2 && val <= range1;
    }
}
