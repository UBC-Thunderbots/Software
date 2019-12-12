#include "software/new_geom/convex_shape.h"
#include "software/new_geom/polygon.h"

/**
 * A polygon that is convex (curved outwards).
 */
class ConvexPolygon : public virtual Polygon, public ConvexShape
{
};
