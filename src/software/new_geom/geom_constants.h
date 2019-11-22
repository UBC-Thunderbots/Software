namespace GeomConstants
{
    // Due to internal representation of doubles being slightly less accurate/consistent
    // with some numbers and operations, we consider doubles that are very close together
    // to be equal (since they likely are, just possibly slightly misrepresented by the
    // system/compiler). We use this EPSILON as a threshold for comparison. 1e-15 was
    // chosen as a value because doubles have about 16 consistent significant figures.
    // Comparing numbers with 15 significant figures gives us a
    // small buffer while remaining as accurate as possible.
    // http://www.cplusplus.com/forum/beginner/95128/
    static constexpr double EPSILON = 1e-15;
}  // namespace GeomConstants
