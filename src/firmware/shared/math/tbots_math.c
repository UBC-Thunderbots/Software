#include "firmware/shared/math/tbots_math.h"

#include <assert.h>

float shared_tbots_math_linearInterpolation(const float x0, const float y0,
                                            const float x1, const float y1,
                                            const float xp)
{
    // Check that the input x values are not identical
    // Identical x values leads to dividing by zero
    assert(x0 != x1);

    return y0 + ((y1 - y0) / (x1 - x0)) * (xp - x0);
}