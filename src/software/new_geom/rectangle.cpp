#include "software/new_geom/rectangle.h"

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

bool Rectangle::contains(const Point &p) const
{
    return p.x() >= negXNegYCorner().x() && p.y() >= negXNegYCorner().y() &&
           p.x() <= negXNegYCorner().x() + diagonal().x() &&
           p.y() <= negXNegYCorner().y() + diagonal().y();
}

Point Rectangle::furthestCorner(const Point &p) const
{
    std::vector<Point> corners = points_;

    return *std::max_element(
        corners.begin(), corners.end(), [&](const Point &corner1, const Point &corner2) {
            return p.distanceFromPoint(corner1) < p.distanceFromPoint(corner2);
        });
}

bool Rectangle::inflate(double amount)
{
    // Ensures rectangle cannot be shrunk to less than a point
    if (xLength() < -2 * amount || yLength() < -2 * amount)
    {
        return false;
    }

    points_[0] = points_[0] + Vector(-amount, -amount);
    points_[1] = points_[1] + Vector(-amount, amount);
    points_[2] = points_[2] + Vector(amount, amount);
    points_[3] = points_[3] + Vector(amount, -amount);

    return true;
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
