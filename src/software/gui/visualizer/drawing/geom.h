#pragma once

#include <QtCore/QLine>
#include <QtCore/QPoint>
#include <QtCore/QRect>
#include <QtGui/QPolygonF>
#include <QtWidgets/QGraphicsScene>

#include "software/gui/visualizer/geom/geometry_conversion.h"
#include "software/new_geom/circle.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/rectangle.h"
#include "software/new_geom/segment.h"

/**
 * Draws the Rectangle on the given scene.
 *
 * @param scene The scene to draw on
 * @param rectangle The Rectangle to draw
 * @param pen The QPen to draw the Rectangle
 * @param brush_opt The optional pointer to a brush to fill the Rectangle
 */
void drawRectangle(QGraphicsScene* scene, const Rectangle& rectangle, const QPen& pen,
                   std::optional<QBrush> brush_opt = std::nullopt);

/**
 * Draws the Polygon on the given scene.
 *
 * @param scene The scene to draw on
 * @param polygon The Polygon to draw
 * @param pen The QPen to draw the Polygon
 * @param brush_opt The optional pointer to a brush to fill the Polygon
 */
void drawPolygon(QGraphicsScene* scene, const Polygon& polygon, const QPen& pen,
                 std::optional<QBrush> brush_opt = std::nullopt);

/**
 * Draws the Circle on the given scene.
 *
 * @param scene The scene to draw on
 * @param circle The Circle to draw
 * @param pen The QPen to draw the Circle
 * @param brush_opt The optional pointer to a brush to fill the Circle
 */
void drawCircle(QGraphicsScene* scene, const Circle& circle, const QPen& pen,
                std::optional<QBrush> brush_opt = std::nullopt);

/**
 * Draws the Segment on the given scene.
 *
 * @param scene The scene to draw on
 * @param segment The Segment to draw
 * @param pen The QPen to draw the Segment
 */
void drawSegment(QGraphicsScene* scene, const Segment& segment, const QPen& pen);
