#include "firmware/shared/math/tbots_math.h"

#include <assert.h>
#include <math.h>

float shared_tbots_math_linearInterpolation(const float x0, const float y0,
                                            const float x1, const float y1, float xp)
{
    // Check that the input x values are not identical
    // Identical x values leads to dividing by zero
    assert(x0 != x1);

    // Clamp the xp variable to be within the interpolation region
    const float min_x = fminf(x0, x1);
    const float max_x = fmaxf(x0, x1);
    if (xp > max_x)
    {
        xp = x1;
    }
    else if (xp < min_x)
    {
        xp = x0;
    }

    return y0 + ((y1 - y0) / (x1 - x0)) * (xp - x0);
}
