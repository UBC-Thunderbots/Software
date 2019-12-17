#include "software/new_geom/convex_shape.h"
#include "software/new_geom/polygon.h"

/**
 * A polygon that is convex (curved outwards).
 */
class ConvexPolygon : public Polygon, public ConvexShape
{
   public:
    ConvexPolygon() = delete;
    /**
     * Returns the area of this convex polygon
     *
     * @return The area of this convex polygon
     */
    double area() const override;

   protected:
    /**
     * Construct a convex polygon by drawing line segments between consecutive
     * Points, and then from the last Point to the first Point.
     *
     * @param points Points that form a convex polygon
     */
    explicit ConvexPolygon(const std::initializer_list<Point>& points);
};
