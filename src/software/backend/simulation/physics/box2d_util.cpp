#include "software/backend/simulation/physics/box2d_util.h"

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
    ret.Set(point.x(), point.y());
    return ret;
}

b2Vec2 createVec2(const Vector& vector)
{
    b2Vec2 ret;
    ret.Set(vector.x(), vector.y());
    return ret;
}
