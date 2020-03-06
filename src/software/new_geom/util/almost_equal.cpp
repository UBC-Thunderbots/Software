#include "software/new_geom/util/almost_equal.h"

#include <algorithm>
#include <cmath>

#include "software/new_geom/geom_constants.h"

// See
// https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
// for detailed explanation
bool almostEqual(double a, double b)
{
    // Check if the numbers are really close -- needed when comparing numbers near zero
    double diff = std::fabs(a - b);
    if (diff < GeomConstants::EPSILON)
    {
        return true;
    }

    a              = fabs(a);
    b              = fabs(b);
    double largest = std::max(a, b);

    if (diff < largest * GeomConstants::EPSILON)
    {
        return true;
    }
    return false;
}
