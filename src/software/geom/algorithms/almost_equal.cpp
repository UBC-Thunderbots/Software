#include "software/geom/algorithms/almost_equal.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "software/geom/geom_constants.h"

int64_t ulpsDistance(double a, double b)
{
    // Save work if the doubles are equal
    // Also handles +0 == -0
    if (a == b)
        return 0;

    const auto max = std::numeric_limits<int64_t>::max();

    // Max distance for NaN
    if (std::isnan(a) || std::isnan(b))
        return max;

    // If one's infinite and they're not equal, max distance
    if (std::isinf(a) || std::isinf(b))
        return max;

    int64_t ia, ib;
    std::memcpy(&ia, &a, sizeof(double));
    std::memcpy(&ib, &b, sizeof(double));

    // Don't compare differently-signed doubles
    if ((ia < 0) != (ib < 0))
        return max;

    // Return the absolute value of the distance in ULPs.
    int64_t distance = ia - ib;
    if (distance < 0)
        distance = -distance;
    return distance;
}

bool almostEqual(double a, double b, double fixed_epsilon, int ulps_epsilon)
{
    // Handle the near-zero case
    const double difference = fabs(a - b);
    if (difference <= fixed_epsilon)
        return true;

    return ulpsDistance(a, b) <= ulps_epsilon;
}
