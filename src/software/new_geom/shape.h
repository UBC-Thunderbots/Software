#include "software/new_geom/point.h"

class IShape
{
   public:
    virtual ~IShape() = default;

    /**
     * Determines whether the given Point is contained within this Shape.
     *
     * @return whether the Point p is contained within this Shape.
     */
    virtual bool contains(const Point &p) const = 0;

    /**
     * Returns the area of this Shape.
     *
     * @return The area of this Shape.
     */
    virtual double area() const = 0;
};
