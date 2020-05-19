#pragma once

#include <QtCore/QLine>
#include <QtCore/QPoint>
#include <QtCore/QRect>
#include <QtGui/QPolygonF>
#include <QtWidgets/QGraphicsScene>

#include "software/new_geom/circle.h"
#include "software/new_geom/convex_polygon.h"
#include "software/new_geom/rectangle.h"
#include "software/new_geom/segment.h"
#include "software/visualizer/geom/geometry_conversion.h"

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
 * Draws the ConvexPolygon on the given scene.
 *
 * @param scene The scene to draw on
 * @param convex_polygon The ConvexPolygon to draw
 * @param pen The QPen to draw the ConvexPolygon
 * @param brush_opt The optional pointer to a brush to fill the ConvexPolygon
 */
void drawConvexPolygon(QGraphicsScene* scene, const ConvexPolygon& convex_polygon,
                       const QPen& pen, std::optional<QBrush> brush_opt = std::nullopt);

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
