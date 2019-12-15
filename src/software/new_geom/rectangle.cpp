#include "software/new_geom/rectangle.h"

Rectangle::Rectangle(const Point &point1, const Point &point2)
    : Polygon({Point(point1.x() < point2.x() ? point1.x() : point2.x(),
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

double Rectangle::area() const
{
    return diagonal().x() * diagonal().y();
}

Point Rectangle::centre() const
{
    return Point(negXNegYCorner() + (diagonal() / 2));
}

Point Rectangle::posXPosYCorner() const
{
    return points_[2];
}

Point Rectangle::negXPosYCorner() const
{
    return points_[1];
}

Point Rectangle::negXNegYCorner() const
{
    return points_[0];
}

Point Rectangle::posXNegYCorner() const
{
    return points_[3];
}

bool Rectangle::contains(const Point &p) const
{
    return p.x() >= negXNegYCorner().x() && p.y() >= negXNegYCorner().y() &&
           p.x() <= negXNegYCorner().x() + diagonal().x() &&
           p.y() <= negXNegYCorner().y() + diagonal().y();
}

Point Rectangle::furthestCorner(const Point &p)
{
    std::vector<Point> corners = points_;

    return *std::max_element(
        corners.begin(), corners.end(), [&](const Point &corner1, const Point &corner2) {
            return p.distanceFromPoint(corner1) < p.distanceFromPoint(corner2);
        });
}

bool Rectangle::operator==(const Rectangle &p) const
{
    return negXNegYCorner() == p.negXNegYCorner() && diagonal() == p.diagonal();
}
