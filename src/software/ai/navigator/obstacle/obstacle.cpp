#include "software/ai/navigator/obstacle/obstacle.hpp"

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

TbotsProto::Obstacle createObstacleProto(const Stadium &stadium)
{
    TbotsProto::Obstacle obstacle_proto;
    *(obstacle_proto.mutable_stadium()) = *createStadiumProto(stadium);
    return obstacle_proto;
}