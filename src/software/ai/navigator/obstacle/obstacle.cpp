#include "software/ai/navigator/obstacle/obstacle.hpp"

TbotsProto::Obstacles createObstacleProto(const Polygon &polygon)
{
    TbotsProto::Obstacles obstacle_proto;
    *(obstacle_proto.add_polygon()) = *createPolygonProto(polygon);
    return obstacle_proto;
}

TbotsProto::Obstacles createObstacleProto(const Rectangle &rectangle)
{
    TbotsProto::Obstacles obstacle_proto;
    *(obstacle_proto.add_polygon()) = *createPolygonProto(rectangle);
    return obstacle_proto;
}

TbotsProto::Obstacles createObstacleProto(const Circle &circle)
{
    TbotsProto::Obstacles obstacle_proto;
    *(obstacle_proto.add_circle()) = *createCircleProto(circle);
    return obstacle_proto;
}

TbotsProto::Obstacles createObstacleProto(const Stadium &stadium)
{
    TbotsProto::Obstacles obstacle_proto;
    *(obstacle_proto.add_stadium()) = *createStadiumProto(stadium);
    return obstacle_proto;
}