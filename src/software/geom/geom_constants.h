#pragma once

// Due to the internal representation of doubles, comparing them to zero using ULPs
// does not make sense. Instead, we have to pick some fixed epsilon to compare doubles
// to zero. We pick 1e-15, which is approximately 10 ULPs near 1.0, and should fit the
// needs of operations near such scale. This is not meant to cover all use cases, and
// other epsilon values may need to be chosen for specific use cases.
// Further reading: https://bitbashing.io/comparing-floats.html
static constexpr double FIXED_EPSILON = 1e-15;

// A single double arithmetic operation with accurate inputs can have a maximum error
// of 0.5 ULP. These errors accumulate with successive operations, and the appropriate
// max ULPs value should be chosen based on the specific use case.
// Further reading: https://bitbashing.io/comparing-floats.html
static constexpr int ULPS_EPSILON_TEN     = 10;
static constexpr int ULPS_EPSILON_HUNDRED = 100;
