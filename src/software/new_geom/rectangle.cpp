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
    return negXNegYCorner() + diagonal();
}

Point Rectangle::negXPosYCorner() const
{
    return negXNegYCorner() + Vector(0, diagonal().y());
}

Point Rectangle::negXNegYCorner() const
{
    return points_[0];
}

Point Rectangle::posXNegYCorner() const
{
    return negXNegYCorner() + Vector(diagonal().x(), 0);
}

bool Rectangle::contains(const Point& p) const
{
    return p.x() >= negXNegYCorner().x() && p.y() >= negXNegYCorner().y() &&
           p.x() <= negXNegYCorner().x() + diagonal().x() &&
           p.y() <= negXNegYCorner().y() + diagonal().y();
}

Point Rectangle::operator[](unsigned int pos) const
{
    switch (pos)
    {
        case 0:
            return negXNegYCorner();
        case 1:
            return negXPosYCorner();
        case 2:
            return posXPosYCorner();
        case 3:
            return posXNegYCorner();
        default:
            throw std::out_of_range("Rectangle only has 4 vertices.");
    }
}

Point Rectangle::furthestCorner(const Point &p)
{
    std::vector<Point> corners = points_;

    return *std::max_element(corners.begin(), corners.end(),
                             [&](const Point& corner1, const Point& corner2) {
                                 return p.distanceFromPoint(corner1) < p.distanceFromPoint(corner2);
                             });
}

bool Rectangle::operator==(const Rectangle &p) const
{
    return negXNegYCorner() == p.negXNegYCorner() && diagonal() == p.diagonal();
}
