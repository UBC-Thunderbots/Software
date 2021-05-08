#pragma once

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/rasterize.h"

template <typename GEOM_TYPE>
GeomObstacle<GEOM_TYPE>::GeomObstacle(const GEOM_TYPE& geom) : geom_(geom)
{
}

template <typename GEOM_TYPE>
bool GeomObstacle<GEOM_TYPE>::contains(const Point& p) const
{
    return ::contains(geom_, p);
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
std::vector<Point> GeomObstacle<GEOM_TYPE>::rasterize(double resolution_size) const
{
    return ::rasterize(geom_, resolution_size);
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
void GeomObstacle<GEOM_TYPE>::accept(ObstacleVisitor& visitor) const
{
    visitor.visit(*this);
}
