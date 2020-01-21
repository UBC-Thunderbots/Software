#include "physics_world.h"

#include "software/backend/simulation/physics/physics_ball.h"
#include "software/backend/simulation/physics/physics_field.h"
#include "software/backend/simulation/physics/physics_robot.h"
#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

PhysicsWorld::PhysicsWorld(const World& world)
{
    b2Vec2 gravity(0, 0);
    b2_world          = std::make_shared<b2World>(gravity);
    current_timestamp = Timestamp::fromSeconds(0);

    initWorld(world);
}

void PhysicsWorld::initWorld(const World& world)
{
    physics_field = std::make_shared<PhysicsField>(b2_world, world.field());
    physics_ball  = std::make_shared<PhysicsBall>(b2_world, world.ball());
    friendly_physics_robots.clear();
    for (const auto& friendly_robot : world.friendlyTeam().getAllRobots())
    {
        friendly_physics_robots.emplace_back(
            std::make_shared<PhysicsRobot>(b2_world, friendly_robot));
    }
    enemy_physics_robots.clear();
    for (const auto& enemy_robot : world.enemyTeam().getAllRobots())
    {
        enemy_physics_robots.emplace_back(
            std::make_shared<PhysicsRobot>(b2_world, enemy_robot));
    }
    current_timestamp = world.getMostRecentTimestamp();
}

World PhysicsWorld::getWorld() const
{
    World new_world;
    new_world.updateTimestamp(current_timestamp);
    if (physics_ball)
    {
        new_world.mutableBall() = physics_ball->getBallWithTimestamp(current_timestamp);
    }
    if (physics_field)
    {
        new_world.mutableField() =
            physics_field->getFieldWithTimestamp(current_timestamp);
    }

    new_world.mutableFriendlyTeam().clearAllRobots();
    std::vector<Robot> friendly_robots;
    for (const auto& robot : friendly_physics_robots)
    {
        friendly_robots.emplace_back(robot->getRobotWithTimestamp(current_timestamp));
    }
    new_world.mutableFriendlyTeam().updateRobots(friendly_robots);

    new_world.mutableEnemyTeam().clearAllRobots();
    std::vector<Robot> enemy_robots;
    for (const auto& robot : enemy_physics_robots)
    {
        enemy_robots.emplace_back(robot->getRobotWithTimestamp(current_timestamp));
    }
    new_world.mutableEnemyTeam().updateRobots(enemy_robots);

    return new_world;
}

void PhysicsWorld::step(const Duration& time_step)
{
    b2_world->Step(time_step.getSeconds(), velocity_iterations, position_iterations);
    current_timestamp = current_timestamp + time_step;
}

std::vector<std::shared_ptr<PhysicsRobot>> PhysicsWorld::getFriendlyPhysicsRobots() const
{
    return friendly_physics_robots;
}

std::vector<std::shared_ptr<PhysicsRobot>> PhysicsWorld::getEnemyPhysicsRobots() const
{
    return enemy_physics_robots;
}

std::shared_ptr<PhysicsBall> PhysicsWorld::getPhysicsBall() const
{
    return physics_ball;
}

std::shared_ptr<PhysicsField> PhysicsWorld::getPhysicsField() const
{
    return physics_field;
}
