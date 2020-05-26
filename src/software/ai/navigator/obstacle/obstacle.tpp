#include "software/ai/navigator/obstacle/obstacle.h"

template <typename GEOM_TYPE>
GeomObstacle<GEOM_TYPE>::GeomObstacle(const GEOM_TYPE& geom) : geom_(geom)
{
}

template <typename GEOM_TYPE>
bool GeomObstacle<GEOM_TYPE>::contains(const Point& p) const
{
    return geom_.contains(p);
}

template <typename GEOM_TYPE>
double GeomObstacle<GEOM_TYPE>::distance(const Point& p) const
{
    return ::distance(geom_, p);
}

template <typename GEOM_TYPE>
bool GeomObstacle<GEOM_TYPE>::intersects(const Segment& segment) const
{
    return ::intersects(geom_, segment);
}

template <typename GEOM_TYPE>
std::string GeomObstacle<GEOM_TYPE>::toString(void) const
{
    std::ostringstream ss;
    ss << "Obstacle with shape " << geom_;
    return ss.str();
}

template <typename GEOM_TYPE>
const GEOM_TYPE GeomObstacle<GEOM_TYPE>::getGeom(void) const
{
    return geom_;
}

template <typename GEOM_TYPE>
std::ostream& operator<<(std::ostream& os, const GeomObstacle<GEOM_TYPE>& geom_obstacle)
{
    os << geom_obstacle.toString();
    return os;
}

std::ostream& operator<<(std::ostream& os, const ObstaclePtr& obstacle_ptr)
{
    os << obstacle_ptr->toString();
    return os;
}
