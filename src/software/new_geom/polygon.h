#include "software/new_geom/shape.h"
#include "software/new_geom/point.h"

class Polygon : public IShape
{
   public:
    Polygon() = delete;

   private:
    std::vector<Segment> segments;
    std::vector<Point> points;
};
