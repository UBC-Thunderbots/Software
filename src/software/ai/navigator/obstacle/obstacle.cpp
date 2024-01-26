#include "software/ai/navigator/obstacle/obstacle.hpp"

#include "proto/message_translation/tbots_geometry.h"

TbotsProto::Obstacle createObstacleProto(const Polygon &polygon)
{
    TbotsProto::Obstacle obstacle_proto;
    *(obstacle_proto.mutable_polygon()) = *createPolygonProto(polygon);
    return obstacle_proto;
}

TbotsProto::Obstacle createObstacleProto(const Rectangle &rectangle)
{
    TbotsProto::Obstacle obstacle_proto;
    *(obstacle_proto.mutable_polygon()) = *createPolygonProto(rectangle);
    return obstacle_proto;
}

TbotsProto::Obstacle createObstacleProto(const Circle &circle)
{
    TbotsProto::Obstacle obstacle_proto;
    *(obstacle_proto.mutable_circle()) = *createCircleProto(circle);
    return obstacle_proto;
}
