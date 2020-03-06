#include "software/new_geom/util/collinear.h"

bool collinear(const Point &a, const Point &b, const Point &c)
{
    if ((a - b).lengthSquared() < GeomConstants::EPSILON ||
        (b - c).lengthSquared() < GeomConstants::EPSILON ||
        (a - c).lengthSquared() < GeomConstants::EPSILON)
    {
        return true;
    }

    std::cout << "collinear results: " << std::abs((b - a).cross(c - a)) << std::endl;
    std::cout << "1: " << b << ", " << a << ", " << b-a << std::endl;
    std::cout << "2: " << c << ", " << a << ", " << c-a << std::endl;
    std::cout << (b - a).cross(c - a) << std::endl;
    return std::fabs((b - a).cross(c - a)) < GeomConstants::EPSILON;
}
