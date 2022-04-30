#include "software/gui/drawing/obstacle_artist.h"

#include "proto/geometry.pb.h"
#include "proto/message_translation/tbots_geometry.h"
#include "software/logger/logger.h"

ObstacleArtist::ObstacleArtist(QGraphicsScene* scene, std::optional<QPen> pen)
    : scene_(scene), pen_(pen), obstacle_proto_()
{
}

void ObstacleArtist::visit(const GeomObstacle<Circle>& geom_obstacle)
{
    // TODO (#2584) This is a nasty hack we need to visualize the obstacles
    // using the obstacle artist.
    //
    // We will be visualizing static obstacles
    // through primitives and dynamic obstacles through the HRVO layer so
    // we can remove this hack.
    if (pen_.has_value())
    {
        drawCircle(scene_, geom_obstacle.getGeom(), pen_.value());
    }
    else
    {
        *(obstacle_proto_.add_circle()) = *createCircleProto(geom_obstacle.getGeom());
    }
}

void ObstacleArtist::visit(const GeomObstacle<Polygon>& geom_obstacle)
{
    // TODO (#2584) This is a nasty hack we need to visualize the obstacles
    if (pen_.has_value())
    {
        drawPolygon(scene_, geom_obstacle.getGeom(), pen_.value());
    }
    else
    {
        *(obstacle_proto_.add_polygon()) = *createPolygonProto(geom_obstacle.getGeom());
    }
}

void ObstacleArtist::visit(const GeomObstacle<Rectangle>& geom_obstacle)
{
    // TODO (#2584) This is a nasty hack we need to visualize the obstacles
    if (pen_.has_value())
    {
        drawPolygon(scene_, geom_obstacle.getGeom(), pen_.value());
    }
    else
    {
        *(obstacle_proto_.add_polygon()) = *createPolygonProto(geom_obstacle.getGeom());
    }
}

void ObstacleArtist::visualize()
{
    // Log and Clear protobuf
    LOG(VISUALIZE) << obstacle_proto_;
    obstacle_proto_ = TbotsProto::Obstacles();
}