#include "software/simulation/physics/box2d_util.h"

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

    // Using the shoelace formula / algorithm from
    // https://www.geeksforgeeks.org/area-of-a-polygon-with-given-n-ordered-vertices/
    // This requires that the vertices are given in order, either clockwise or
    // counter-clockwise.
    double area    = 0.0;
    unsigned int j = polygon.m_count - 1;
    for (int i = 0; i < polygon.m_count; i++)
    {
        double x_sum        = polygon.m_vertices[j].x + polygon.m_vertices[i].x;
        double y_difference = polygon.m_vertices[j].y - polygon.m_vertices[i].y;
        area += x_sum * y_difference;
        j = i;
    }

    return static_cast<float>(std::fabs(area / 2.0));
}
