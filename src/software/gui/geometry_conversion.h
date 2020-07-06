#pragma once

#include <QtCore/QLine>
#include <QtCore/QPoint>
#include <QtCore/QRect>
#include <QtGui/QPolygonF>

#include "software/new_geom/point.h"
#include "software/new_geom/rectangle.h"
#include "software/new_geom/segment.h"

/**
 * Converts our Point class to a QPointF
 * @param point The Point to convert
 * @return The QPointF representation of the given Point
 */
QPointF createQPointF(const Point& point);

/**
 * Converts our Rectangle class to a QRectF
 * @param rectangle The Rectangle to convert
 * @return The QRectF representation of the given Rectangle
 */
QRectF createQRectF(const Rectangle& rectangle);

/**
 * Converts our Polygon class to a QPolygonF
 * @param polygon The Polygon to convert
 * @return The QPolygonF representation of the given Polygon
 */
QPolygonF createQPolygonF(const Polygon& polygon);

/**
 * Converts our Segment class to a QLineF
 * @param segment the Segment to convert
 * @return The QLineF representation of the given Segment
 */
QLineF createQLineF(const Segment& segment);

/**
 * Converts our Angle class to an angle value consumable by Qt.
 * Qt uses integers to represent angles.
 *
 * @param angle The Angle to convert
 *
 * @return The Qt angle representation of the given Angle
 */
// TODO: Test once https://github.com/UBC-Thunderbots/Software/pull/1583 is merged
int createQAngle(const Angle& angle);
