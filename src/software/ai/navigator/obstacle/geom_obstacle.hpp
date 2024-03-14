#pragma once

#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersects.h"
#include "software/geom/algorithms/rasterize.h"

template <typename GEOM_TYPE>
class GeomObstacle : public Obstacle
{
   public:
    GeomObstacle() = delete;

    /**
     * Construct a static GeomObstacle with GEOM_TYPE
     *
     * @param geom GEOM_TYPE to make obstacle with
     */
    explicit GeomObstacle(const GEOM_TYPE& geom);

    bool contains(const Point& p, const double t_sec = 0) const override;
    double distance(const Point& p, const double t_sec = 0) const override;
    double signedDistance(const Point& p, const double t_sec = 0) const override;
    bool intersects(const Segment& segment, const double t_sec = 0) const override;
    Point closestPoint(const Point& p) const override;
    TbotsProto::Obstacle createObstacleProto() const override;
    Rectangle axisAlignedBoundingBox(double inflation_radius = 0) const override;
    std::string toString(void) const override;
    void accept(ObstacleVisitor& visitor) const override;
    std::vector<Point> rasterize(const double resolution_size) const override;

    /**
     * Gets the underlying GEOM_TYPE
     *
     * @return geom type
     */
    const GEOM_TYPE getGeom(void) const;

   protected:
    const GEOM_TYPE geom_;
};


template <typename GEOM_TYPE>
GeomObstacle<GEOM_TYPE>::GeomObstacle(const GEOM_TYPE& geom) : geom_(geom)
{
}

template <typename GEOM_TYPE>
bool GeomObstacle<GEOM_TYPE>::contains(const Point& p, const double t_sec) const
{
    return ::contains(geom_, p);
}

template <typename GEOM_TYPE>
double GeomObstacle<GEOM_TYPE>::distance(const Point& p, const double t_sec) const
{
    return ::distance(geom_, p);
}

template <typename GEOM_TYPE>
double GeomObstacle<GEOM_TYPE>::signedDistance(const Point &p, const double t_sec) const
{
    return ::signedDistance(geom_, p);
}

template <typename GEOM_TYPE>
bool GeomObstacle<GEOM_TYPE>::intersects(const Segment& segment, const double t_sec) const
{
    return ::intersects(geom_, segment);
}

template <typename GEOM_TYPE>
Point GeomObstacle<GEOM_TYPE>::closestPoint(const Point& p) const
{
    return ::closestPoint(geom_, p);
}

template <typename GEOM_TYPE>
std::vector<Point> GeomObstacle<GEOM_TYPE>::rasterize(const double resolution_size) const
{
    return ::rasterize(geom_, resolution_size);
}

template <typename GEOM_TYPE>
TbotsProto::Obstacle GeomObstacle<GEOM_TYPE>::createObstacleProto() const
{
    return ::createObstacleProto(geom_);
}

template <typename GEOM_TYPE>
Rectangle GeomObstacle<GEOM_TYPE>::axisAlignedBoundingBox(
    const double inflation_radius) const
{
    return ::axisAlignedBoundingBox(geom_, inflation_radius);
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
