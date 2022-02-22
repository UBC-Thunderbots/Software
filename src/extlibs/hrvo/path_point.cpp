#include "extlibs/hrvo/path_point.h"
#include "extlibs/hrvo/vector2.h"

#include <stdexcept>
#include <utility>


PathPoint::PathPoint(const Vector2 &position) 
    : position_(position), 
      speed_at_destination(0.0f)
{
}

PathPoint::PathPoint(const Vector2 &position, const float destination_speed) 
    : position_(position), 
      speed_at_destination(destination_speed)      
{
}