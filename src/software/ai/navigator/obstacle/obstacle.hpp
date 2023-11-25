#pragma once

#include <memory>
#include <sstream>
#include <vector>

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive.pb.h"
#include "shared/constants.h"
#include "software/ai/navigator/obstacle/obstacle_visitor.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/generate_velocity_obstacle.h"
#include "software/geom/algorithms/intersects.h"
#include "software/geom/algorithms/rasterize.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"

/**
 * An obstacle is an area to avoid for navigation
 */
class Obstacle
{
   public:
    Obstacle()          = default;
    virtual ~Obstacle() = default;

    /**
     * Determines whether the given Point is contained within this Obstacle
     *
     * @return whether the Point p is contained within this Obstacle
     */
    virtual bool contains(const Point& p) const = 0;

    /**
     * Gets the minimum distance from the obstacle to the point
     *
     * @param point Point to get distance to
     *
     * @return distance to point
     */
    virtual double distance(const Point& p) const = 0;

    /**
     * Determines whether the given Segment intersects this Obstacle
     *
     * @return true if the given Segment intersects this Obstacle
     */
    virtual bool intersects(const Segment& segment) const = 0;

    /**
     * Determines what coordinates on the field are blocked by this Obstacle
     */
    virtual std::vector<Point> rasterize(const double) const = 0;

    /**
     * Creates an obstacle proto representation
     */
    virtual TbotsProto::Obstacles createObstacleProto() const = 0;

    /**
     * Draw a velocity obstacle for the given agent to this obstacle
     */
    virtual VelocityObstacle generateVelocityObstacle(const Circle&,
                                                      const Vector&) const = 0;

    /**
     * Output string to describe the obstacle
     *
     * @return string that describes the obstacle
     */
    virtual std::string toString(void) const = 0;

    /**
     * Accepts an Obstacle Visitor and calls the visit function
     *
     * @param visitor An Obstacle Visitor
     */
    virtual void accept(ObstacleVisitor& visitor) const = 0;
};

/**
 * Creates an obstacle proto from polygon or circle
 *
 * @param the geom object
 * @return the obstacle proto
 */
TbotsProto::Obstacles createObstacleProto(const Polygon& polygon);
TbotsProto::Obstacles createObstacleProto(const Rectangle& rectangle);
TbotsProto::Obstacles createObstacleProto(const Circle& circle);

template <typename GEOM_TYPE>
class GeomObstacle : public Obstacle
{
   public:
    GeomObstacle() = delete;

    /**
     * Construct a GeomObstacle with GEOM_TYPE
     *
     * @param geom GEOM_TYPE to make obstacle with
     */
    explicit GeomObstacle(const GEOM_TYPE& geom);

    bool contains(const Point& p) const override;
    double distance(const Point& p) const override;
    bool intersects(const Segment& segment) const override;
    TbotsProto::Obstacles createObstacleProto() const override;
    std::string toString(void) const override;
    void accept(ObstacleVisitor& visitor) const override;
    std::vector<Point> rasterize(const double resolution_size) const override;
    VelocityObstacle generateVelocityObstacle(const Circle&,
                                              const Vector&) const override;

    /**
     * Gets the underlying GEOM_TYPE
     *
     * @return geom type
     */
    const GEOM_TYPE getGeom(void) const;

   private:
    GEOM_TYPE geom_;
};


/**
 * We use a pointer to Obstacle to support inheritance
 * Note: this is a convenience typedef
 */
using ObstaclePtr = std::shared_ptr<Obstacle>;

/**
 * Implements the << operator for printing
 *
 * @param ostream The stream to print to
 * @param obstacle_ptr The ObstaclePtr to print
 *
 * @return The output stream with the string representation of the class appended
 */
inline std::ostream& operator<<(std::ostream& os, const ObstaclePtr& obstacle_ptr)
{
    os << obstacle_ptr->toString();
    return os;
}

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
std::vector<Point> GeomObstacle<GEOM_TYPE>::rasterize(const double resolution_size) const
{
    return ::rasterize(geom_, resolution_size);
}

template <typename GEOM_TYPE>
TbotsProto::Obstacles GeomObstacle<GEOM_TYPE>::createObstacleProto() const
{
    return ::createObstacleProto(geom_);
}

template <typename GEOM_TYPE>
VelocityObstacle GeomObstacle<GEOM_TYPE>::generateVelocityObstacle(
    const Circle& robot, const Vector& obstacle_velocity) const
{
    return ::generateVelocityObstacle(geom_, robot, obstacle_velocity);
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
