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

Point Rectangle::minCorner() const
{
    return points_[0];
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
    return Point(minCorner() + (diagonal() / 2));
}

Point Rectangle::posXPosYCorner() const
{
    return minCorner() + diagonal();
}

Point Rectangle::negXPosYCorner() const
{
    return minCorner() + Vector(0, diagonal().y());
}

Point Rectangle::negXNegYCorner() const
{
    return minCorner();
}

Point Rectangle::posXNegYCorner() const
{
    return minCorner() + Vector(diagonal().x(), 0);
}

bool Rectangle::contains(const Point& p) const
{
    return p.x() >= minCorner().x() && p.y() >= minCorner().y() &&
           p.x() <= minCorner().x() + diagonal().x() &&
           p.y() <= minCorner().y() + diagonal().y();
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

Point Rectangle::furthestCorner(Point p)
{
    std::vector<Point> corners = this->corners();

    return *std::max_element(corners.begin(), corners.end(),
                             [&](const Point& corner1, const Point& corner2) {
                                 return dist(corner1, p) < dist(corner2, p);
                             });
}

bool Rectangle::operator==(const Rectangle &p) const
{
    return minCorner() == p.minCorner() && diagonal() == p.diagonal();
}
