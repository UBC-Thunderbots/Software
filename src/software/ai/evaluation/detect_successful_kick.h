#pragma once

#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/world/ball.h"


bool successfulKickDetected(const Ball& ball,const Point& kick_origin, const Angle& kick_direction);
