#include "ai/navigator/trespass.h"

#include "geom/circle.h"
#include "geom/point.h"

namespace Navigator
{
    namespace Trespass
    {
        unsigned int calcLinearTrespassScore(Circle circle, Point point)
        {
            if (point.isnan())
                return 0;
            
            // The point is in the circle if the distance to 
            // the center of the circle is less than the radius
            const double dx = point.x() - circle.getOrigin().x();
            const double dy = point.y() - circle.getOrigin().y();
            const double r  = circle.getRadius();

            return dx * dx + dy * dy > r * r;
        }
    }
}  // namespace Navigator
