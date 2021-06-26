#include "software/geom/rectangle.h"

#include <algorithm>

Rectangle::Rectangle(const Point &point1, const Point &point2)
    : ConvexPolygon({Point(point1.x() < point2.x() ? point1.x() : point2.x(),
                           point1.y() < point2.y() ? point1.y() : point2.y()),
                     Point(point1.x() < point2.x() ? point1.x() : point2.x(),
                           point1.y() > point2.y() ? point1.y() : point2.y()),
                     Point(point1.x() > point2.x() ? point1.x() : point2.x(),
                           point1.y() > point2.y() ? point1.y() : point2.y()),
                     Point(point1.x() > point2.x() ? point1.x() : point2.x(),
                           point1.y() < point2.y() ? point1.y() : point2.y())})
{
}

Vector Rectangle::diagonal() const
{
    return points_[2] - points_[0];
}

double Rectangle::xLength() const
{
    return diagonal().x();
}

double Rectangle::yLength() const
{
    return diagonal().y();
}

Point Rectangle::centre() const
{
    return Point(negXNegYCorner() + (diagonal() / 2));
}

const Point &Rectangle::posXPosYCorner() const
{
    return points_[2];
}

const Point &Rectangle::negXPosYCorner() const
{
    return points_[1];
}

const Point &Rectangle::negXNegYCorner() const
{
    return points_[0];
}

const Point &Rectangle::posXNegYCorner() const
{
    return points_[3];
}

double Rectangle::xMax() const
{
    return posXPosYCorner().x();
}
double Rectangle::xMin() const
{
    return negXNegYCorner().x();
}
double Rectangle::yMax() const
{
    return posXPosYCorner().y();
}
double Rectangle::yMin() const
{
    return negXNegYCorner().y();
}

Rectangle Rectangle::expand(const Vector &v) const
{
    Point negCorner = negXNegYCorner();
    Point posCorner = posXPosYCorner();
    if (v.x() > 0)
    {
        posCorner = posCorner + Vector(v.x(), 0);
    }

    if (v.y() > 0)
    {
        posCorner = posCorner + Vector(0, v.y());
    }

    if (v.x() < 0)
    {
        negCorner = negCorner + Vector(v.x(), 0);
    }

    if (v.y() < 0)
    {
        negCorner = negCorner + Vector(0, v.y());
    }
    return Rectangle(negCorner, posCorner);
}

bool Rectangle::operator==(const Rectangle &p) const
{
    return negXNegYCorner() == p.negXNegYCorner() && diagonal() == p.diagonal();
}
