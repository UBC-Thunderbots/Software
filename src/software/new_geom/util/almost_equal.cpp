#include "software/new_geom/util/almost_equal.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "software/new_geom/geom_constants.h"

int64_t ulpsDistance(const double a, const double b);

// See
// https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
// for detailed explanation
bool almostEqual(double a, double b, double fixedEpsilon, int ulpsEpsilon)
{
    // Handle the near-zero case
    const float difference = fabs(a - b);
    if (difference <= fixedEpsilon) return true;

    return ulpsDistance(a, b) <= ulpsEpsilon;
}

int64_t ulpsDistance(const double a, const double b)
{
    // Save work if the doubles are equal
    // Also handles +0 == -0
    if (a == b) return 0;

    const auto max =
            std::numeric_limits<int64_t>::max();

    // Max distance for NaN
    if (std::isnan(a) || std::isnan(b)) return max;

    // If one's infinite and they're not equal, max distance
    if (std::isinf(a) || std::isinf(b)) return max;

    int64_t ia, ib;
    memcpy(&ia, &a, sizeof(double));
    memcpy(&ib, &b, sizeof(double));

    // Don't compare differently-signed doubles
    if ((ia < 0) != (ib < 0)) return max;

    // Return the absolute value of the distance in ULPs.
    int64_t distance = ia - ib;
    if (distance < 0) distance = -distance;
    return distance;
}
