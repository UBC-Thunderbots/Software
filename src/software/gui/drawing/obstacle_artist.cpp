#include "software/gui/drawing/obstacle_artist.h"

#include "proto/geometry.pb.h"
#include "proto/message_translation/tbots_geometry.h"
#include "software/logger/logger.h"

ObstacleArtist::ObstacleArtist(QGraphicsScene* scene, const QPen& pen)
    : scene_(scene), pen_(pen), obstacle_proto_()
{
}

void ObstacleArtist::visit(const GeomObstacle<Circle>& geom_obstacle)
{
    *(obstacle_proto_.add_circle()) = *createCircleProto(geom_obstacle.getGeom());
    drawCircle(scene_, geom_obstacle.getGeom(), pen_);
}

void ObstacleArtist::visit(const GeomObstacle<Polygon>& geom_obstacle)
{
    *(obstacle_proto_.add_polygon()) = *createPolygonProto(geom_obstacle.getGeom());
    drawPolygon(scene_, geom_obstacle.getGeom(), pen_);
}

void ObstacleArtist::visit(const GeomObstacle<Rectangle>& geom_obstacle)
{
    *(obstacle_proto_.add_polygon()) = *createPolygonProto(geom_obstacle.getGeom());
    drawPolygon(scene_, geom_obstacle.getGeom(), pen_);
}

void ObstacleArtist::visualize()
{
    // Log and Clear protobuf
    obstacle_proto_ = TbotsProto::Obstacles();
}
