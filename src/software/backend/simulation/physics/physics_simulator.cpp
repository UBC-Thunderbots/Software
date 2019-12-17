#include "software/backend/simulation/physics/physics_simulator.h"

#include <Box2D/Box2D.h>

PhysicsSimulator::PhysicsSimulator(const World& world)
{
    b2Vec2 gravity(0, 0);
    physics_world           = std::make_shared<b2World>(gravity);
    physics_world_timestamp = Timestamp::fromSeconds(0);

    setWorld(world);
}

World PhysicsSimulator::stepSimulation(const Duration& time_step)
{
    physics_world->Step(time_step.getSeconds(), velocity_iterations, position_iterations);
    physics_world_timestamp = physics_world_timestamp + time_step;

    World new_world;
    new_world.updateTimestamp(physics_world_timestamp);
    if (physics_ball)
    {
        new_world.mutableBall() =
            physics_ball->getBallWithTimestamp(physics_world_timestamp);
    }
    if (physics_field)
    {
        new_world.mutableField() =
            physics_field->getFieldWithTimestamp(physics_world_timestamp);
    }

    return new_world;
}

void PhysicsSimulator::setWorld(const World& world)
{
    physics_field = std::make_unique<PhysicsField>(physics_world, world.field());
    physics_ball  = std::make_unique<PhysicsBall>(physics_world, world.ball());
    physics_world_timestamp = world.getMostRecentTimestamp();
}
