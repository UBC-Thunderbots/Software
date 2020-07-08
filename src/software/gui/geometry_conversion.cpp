#include "software/gui/geometry_conversion.h"

QPointF createQPointF(const Point& point)
{
    return QPointF(point.x(), point.y());
}

QRectF createQRectF(const Rectangle& rectangle)
{
    // In Qt's default coordinate system, (0, 0) is the "top-left" of the screen,
    // y increases downwards, and x increases to the right. This means the "top-left"
    // corner of a rectangle is the corner with the most negative x and y coordinate,
    // and "bottom-right" is positive x and positive y
    return QRectF(createQPointF(rectangle.negXNegYCorner()),
                  createQPointF(rectangle.posXPosYCorner()));
}

QPolygonF createQPolygonF(const Polygon& polygon)
{
    std::vector<QPointF> qpoints;
    for (const auto& p : polygon.getPoints())
    {
        qpoints.push_back(createQPointF(p));
    }
    return QPolygonF(QVector<QPointF>::fromStdVector(qpoints));
}

QLineF createQLineF(const Segment& segment)
{
    return QLineF(createQPointF(segment.getStart()), createQPointF(segment.getEnd()));
}

Point createPoint(const QPointF& point)
{
    return Point(point.x(), point.y());
}

Point createPoint(const QPoint& point)
{
    return Point(static_cast<double>(point.x()), static_cast<double>(point.y()));
}

int createQAngle(const Angle& angle)
{
    // Qt uses integers to represent angles, in 16ths of a degree
    // https://doc.qt.io/qt-5/qgraphicsellipseitem.html#spanAngle
    // Qt's default coordinate convention also means that positive
    // angles / rotation appear clockwise on the screen, as opposed
    // to our convention of positive rotation being counter-clockwise.
    // This is why we negate the value, to reverse the rotation to match
    // our convention.
    return -static_cast<int>(angle.toDegrees() * 16);
}
