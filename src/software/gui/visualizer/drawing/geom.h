#pragma once

#include "../../../../../../.cache/bazel/_bazel_cody/bc67860d71ef5a6fe8384091c58fcd52/execroot/__main__/external/qt/QtCore/QLine"
#include "../../../../../../.cache/bazel/_bazel_cody/bc67860d71ef5a6fe8384091c58fcd52/execroot/__main__/external/qt/QtCore/QPoint"
#include "../../../../../../.cache/bazel/_bazel_cody/bc67860d71ef5a6fe8384091c58fcd52/execroot/__main__/external/qt/QtCore/QRect"
#include "../../../../../../.cache/bazel/_bazel_cody/bc67860d71ef5a6fe8384091c58fcd52/execroot/__main__/external/qt/QtGui/QPolygonF"
#include "../../../../../../.cache/bazel/_bazel_cody/bc67860d71ef5a6fe8384091c58fcd52/execroot/__main__/external/qt/QtWidgets/QGraphicsScene"

#include "software/new_geom/circle.h"
#include "software/new_geom/polygon.h"
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
