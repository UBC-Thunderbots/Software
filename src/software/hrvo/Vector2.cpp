#include "Vector2.h"

#include <ostream>

std::ostream &operator<<(std::ostream &stream, const Vector2 &vector)
{
    stream << vector.getX() << "," << vector.getY();

    return stream;
}
