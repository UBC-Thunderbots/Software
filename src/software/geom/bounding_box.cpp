#include "software/geom/bounding_box.h"

BoundingBox::BoundingBox(const Point &point1, const Point &point2):
        pos_x_pos_y_corner(std::max(point1.x(), point2.x()), std::max(point1.y(), point2.y())),
        neg_x_neg_y_corner(std::min(point1.x(), point2.x()), std::min(point1.y(), point2.y()))
{}

const Point &BoundingBox::posXPosYCorner() const
{
    return pos_x_pos_y_corner;
}

const Point &BoundingBox::negXNegYCorner() const
{
    return neg_x_neg_y_corner;
}

double BoundingBox::xMax() const
{
    return posXPosYCorner().x();
}
double BoundingBox::xMin() const
{
    return negXNegYCorner().x();
}
double BoundingBox::yMax() const
{
    return posXPosYCorner().y();
}
double BoundingBox::yMin() const
{
    return negXNegYCorner().y();
}
