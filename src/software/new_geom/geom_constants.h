#pragma once

namespace GeomConstants
{
    // Due to the internal representation of doubles, comparing them to zero using ULPs
    // does not make sense. Instead, we have to pick some fixed epsilon to compare doubles
    // to zero. We pick 1e-15, which is approximately 10 ULPs near 1.0, and should fit the
    // needs of operations near such scale. This is not meant to cover all usage case, and
    // other epsilon values may need to be chosen for specific use cases. Further reading
    // on floating-point numbers and their comparisons:
    // https://bitbashing.io/comparing-floats.html
    static constexpr double FIXED_EPSILON_ONE = 1e-15;

    // A single double arithmetic operation with accurate inputs can have a maximum error
    // of 0.5 ULP. These errors accumulate with successive operations, and the appropriate
    // max ULPs value should be chosen based on the specific use case.
    static constexpr int MAX_ULPS_TEN     = 10;
    static constexpr int MAX_ULPS_HUNDRED = 100;
}  // namespace GeomConstants
