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

double Rectangle::halfPerimeter() const
{
    return xLength() + yLength();
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

Rectangle Rectangle::expand(double expansion_amount) const
{
    auto points = Polygon::expand(expansion_amount).getPoints();
    return Rectangle(points[0], points[2]);
}

Rectangle Rectangle::shrink(double inset_amount) const
{
    if (inset_amount < 0 || inset_amount > std::max(xLength() / 2, yLength() / 2))
    {
        throw std::invalid_argument(
            "Rectangle::shrink: inset_amount must be non-negative and"
            "less than or equal to min(xLength / 2, yLength / 2)");
    }

    return Rectangle(Point(xMin() + inset_amount, yMin() + inset_amount),
                     Point(xMax() - inset_amount, yMax() - inset_amount));
}

bool Rectangle::operator==(const Rectangle &p) const
{
    return negXNegYCorner() == p.negXNegYCorner() && diagonal() == p.diagonal();
}
