#include "software/gui/drawing/obstacle_artist.h"

#include "proto/geometry.pb.h"
#include "proto/message_translation/tbots_geometry.h"
#include "software/logger/logger.h"

ObstacleArtist::ObstacleArtist(QGraphicsScene* scene, const QPen& pen)
    : scene_(scene), pen_(pen)
{
}

void ObstacleArtist::visit(const GeomObstacle<Circle>& geom_obstacle)
{
    TbotsProto::Circle circle_proto;
    auto circle = geom_obstacle.getGeom();

    *(circle_proto.mutable_origin()) = *createPointProto(circle.origin());
    circle_proto.set_radius(circle.radius());
    *(obstacle_proto_.add_circle()) = circle_proto;

    drawCircle(scene_, geom_obstacle.getGeom(), pen_);
}

void ObstacleArtist::visit(const GeomObstacle<Polygon>& geom_obstacle)
{
    TbotsProto::Polygon poly_proto;
    std::vector<Point> points = geom_obstacle.getGeom().getPoints();
    for (const auto& point : points)
    {
        *(poly_proto.add_points()) = *createPointProto(point);
    }
    *(obstacle_proto_.add_polygon()) = poly_proto;
    drawPolygon(scene_, geom_obstacle.getGeom(), pen_);
}

void ObstacleArtist::visit(const GeomObstacle<Rectangle>& geom_obstacle)
{
    TbotsProto::Polygon poly_proto;
    std::vector<Point> points = geom_obstacle.getGeom().getPoints();
    for (const auto& point : points)
    {
        *(poly_proto.add_points()) = *createPointProto(point);
    }
    *(obstacle_proto_.add_polygon()) = poly_proto;
    drawPolygon(scene_, geom_obstacle.getGeom(), pen_);
}

void ObstacleArtist::visualize()
{
    // Log and Clear protobuf
    LOG(VISUALIZE) << obstacle_proto_;
    obstacle_proto_ = TbotsProto::Obstacle();
}
