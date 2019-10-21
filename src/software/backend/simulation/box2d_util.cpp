#include "software/backend/simulation/box2d_util.h"

bool bodyExistsInWorld(b2Body* body, std::shared_ptr<b2World> world)
{
    if (body == nullptr || !world)
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
