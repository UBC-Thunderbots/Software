#pragma once

#include <QtWidgets/QGraphicsScene>

#include "../../ai/navigator/navigator.h"
#include "../../ai/navigator/obstacle/obstacle.h"
#include "../geometry_conversion/geometry_conversion.h"
#include "colors.h"
#include "draw_functions.h"
#include "geom.h"
#include "obstacle_artist.h"

/**
 * Gets the draw function for drawing the navigator
 *
 * @param navigator The Navigator to draw
 */
AIDrawFunction drawNavigator(std::shared_ptr<Navigator> navigator);
