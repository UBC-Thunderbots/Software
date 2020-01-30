#include "software/backend/simulation/physics/physics_object.h"

PhysicsObject::PhysicsObject(PhysicsObjectType object_type,
                             PhysicsObject *physics_object_ptr)
    : user_data(std::make_unique<PhysicsObjectUserData>(object_type, physics_object_ptr))
{
}
