#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/ai/navigator/navigator.h"
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
 * Draws the ObstaclePtr on the given scene.
 *
 * @param scene The scene to draw on
 * @param obstacle_ptr The ObstaclePtr to draw
 * @param pen The QPen to draw the obstacle
 */
void drawObstacle(QGraphicsScene* scene, const ObstaclePtr& obstacle_ptr,
                  const QPen& pen);
