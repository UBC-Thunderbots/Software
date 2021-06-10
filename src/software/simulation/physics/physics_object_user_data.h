#pragma once

/**
 * Different types of physics objects that exist in the simulation world
 */
enum class PhysicsObjectType
{
    ROBOT_CHICKER,
    ROBOT_DRIBBLER,
    ROBOT_BODY,
    BALL,
    FIELD_WALL,
};

/**
 * This struct contains all the user data we link to Box2D physics objects and fixtures.
 * It gives us a way to refer back to our "Physics..." objects when working with Box2D
 * APIs
 *
 * See https://www.iforce2d.net/b2dtut/user-data for more info about Box2D user data
 */
struct PhysicsObjectUserData
{
    PhysicsObjectUserData(PhysicsObjectType type, void* object_ptr)
        : type(type), physics_object(object_ptr)
    {
    }
    //    PhysicsObjectUserData() = delete;

    PhysicsObjectType type;
    void* physics_object;
};
