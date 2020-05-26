#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/ai/navigator/navigator.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/visualizer/drawing/colors.h"
#include "software/visualizer/drawing/draw_functions.h"
#include "software/visualizer/drawing/geom.h"
#include "software/visualizer/drawing/obstacle_artist.h"
#include "software/visualizer/geom/geometry_conversion.h"

/**
 * Gets the draw function for drawing the navigator
 *
 * @param navigator The Navigator to draw
 */
AIDrawFunction drawNavigator(std::shared_ptr<Navigator> navigator);
