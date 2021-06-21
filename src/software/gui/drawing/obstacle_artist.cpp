#include "software/gui/drawing/obstacle_artist.h"

ObstacleArtist::ObstacleArtist(QGraphicsScene* scene, const QPen& pen)
    : scene_(scene), pen_(pen)
{
}

void ObstacleArtist::visit(const GeomObstacle<Circle>& geom_obstacle)
{
    // drawCircle(scene_, geom_obstacle.getGeom(), pen_);
}

void ObstacleArtist::visit(const GeomObstacle<Polygon>& geom_obstacle)
{
    // drawPolygon(scene_, geom_obstacle.getGeom(), pen_);
}
