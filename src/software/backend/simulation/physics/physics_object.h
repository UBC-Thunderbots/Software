#pragma once

#include <memory>

class PhysicsObject;

/**
 * Different types of physics objects that exist in the simulation world
 */
enum PhysicsObjectType {
    ROBOT_CHICKER,
    ROBOT_DRIBBLER,
    BALL
};

/**
 * This struct contains all the user data we link to Box2D physics objects and fixtures.
 * It gives us a way to refer back to our "Physics..." objects when working with Box2D APIs
 *
 * See https://www.iforce2d.net/b2dtut/user-data for more info about Box2D user data
 */
struct PhysicsObjectUserData {
    PhysicsObjectUserData(PhysicsObjectType type, PhysicsObject* object_ptr) : type(type), physics_object(object_ptr) {}
    PhysicsObjectUserData() = delete;

    PhysicsObjectType type;
    // TODO: maybe weak_ptr?
    PhysicsObject* physics_object;
};

/**
 * This class represents a generic physics object in the simulation world.
 */
class PhysicsObject {
public:
    /**
     * Creates a new PhysicsObject and populates the user_data
     *
     * @param object_type The type of the physics object
     * @param physics_object_ptr A pointer to the instance of the physics object
     */
    explicit PhysicsObject(PhysicsObjectType object_type, PhysicsObject* physics_object_ptr);

    PhysicsObject() = delete;

protected:
    // Every PhysicsObject must provide UserData so that Box2D physics objects and fixtures
    // can link back to our physics objects. Box2D takes user data as a void* and is not
    // responsible for de-allocating the memory. We own a unique_ptr so the de-allocation
    // is handled for us, and we can provide the internal raw pointer to Box2D
    std::unique_ptr<PhysicsObjectUserData> user_data;
};

