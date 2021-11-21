#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/ai/navigator/navigator.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/gui/drawing/colors.h"
#include "software/gui/drawing/draw_functions.h"
#include "software/gui/drawing/geom.h"
#include "software/gui/drawing/obstacle_artist.h"
#include "software/gui/geometry_conversion.h"

/**
 * Gets the draw function for drawing the navigator
 *
 * @param navigator The Navigator to draw
 */
AIDrawFunction drawNavigator(std::shared_ptr<Navigator> navigator);
