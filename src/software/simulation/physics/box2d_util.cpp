#include "software/simulation/physics/box2d_util.h"
#include <vector>
#include "software/geom/convex_polygon.h"

bool bodyExistsInWorld(b2Body* body, b2World* world)
{
    if (body == nullptr || world == nullptr)
    {
        return false;
    }

    b2Body* current_body = world->GetBodyList();
    while (current_body != nullptr)
    {
        if (current_body == body)
        {
            return true;
        }
        current_body = current_body->GetNext();
    }
    return false;
}

b2Vec2 createVec2(const Point& point)
{
    b2Vec2 ret;
    ret.Set(static_cast<float>(point.x()), static_cast<float>(point.y()));
    return ret;
}

b2Vec2 createVec2(const Vector& vector)
{
    b2Vec2 ret;
    ret.Set(static_cast<float>(vector.x()), static_cast<float>(vector.y()));
    return ret;
}

Point createPoint(const b2Vec2& vec)
{
    return Point(vec.x, vec.y);
}

Vector createVector(const b2Vec2& vec)
{
    return Vector(vec.x, vec.y);
}

float polygonArea(const b2PolygonShape& polygon)
{
    // Box2D already asserts that Polygons are not degenerate (have < 3 vertices) when
    // they are created, so we do not need to check for that here.
    std::vector<Point> vertices;
    for (int i = 0; i < polygon.m_count; i++)
    {
        vertices.emplace_back(Point(polygon.m_vertices[i].x, polygon.m_vertices[i].y));
    }
    ConvexPolygon convex_polygon(vertices);

    return static_cast<float>(convex_polygon.area());
}
