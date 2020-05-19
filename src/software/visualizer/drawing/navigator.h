#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/ai/navigator/navigator.h"
#include "software/ai/navigator/obstacle/circle_obstacle.h"
#include "software/ai/navigator/obstacle/convex_polygon_obstacle.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/visualizer/drawing/colors.h"
#include "software/visualizer/drawing/draw_functions.h"
#include "software/visualizer/drawing/geom.h"
#include "software/visualizer/geom/geometry_conversion.h"

/**
 * Gets the draw function for drawing the navigator
 *
 * @param navigator The Navigator to draw
 */
AIDrawFunction drawNavigator(std::shared_ptr<Navigator> navigator);

/**
 * Draws the obstacle on the given scene.
 *
 * @param scene The scene to draw on
 * @param obstacle The ObstaclePtr to draw
 * @param pen The QPen to draw the obstacle
 */
void drawObstacle(QGraphicsScene* scene, const ObstaclePtr obstacle, const QPen& pen);
