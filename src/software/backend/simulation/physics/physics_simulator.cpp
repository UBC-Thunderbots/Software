#include "software/backend/simulation/physics/physics_simulator.h"
#include "software/world/robot.h"

#include <Box2D/Box2D.h>

PhysicsSimulator::PhysicsSimulator(const World& world)
{
    b2Vec2 gravity(0, 0);
    physics_world           = std::make_shared<b2World>(gravity);
    physics_world_timestamp = Timestamp::fromSeconds(0);

    setWorld(world);
}

void PhysicsSimulator::stepSimulation(const Duration& time_step) {
    physics_world->Step(time_step.getSeconds(), velocity_iterations, position_iterations);
    physics_world_timestamp = physics_world_timestamp + time_step;
}

World PhysicsSimulator::getWorld() const {
    World new_world;
    new_world.updateTimestamp(physics_world_timestamp);
    if(physics_ball)
    {
        new_world.mutableBall() =
                physics_ball->getBallWithTimestamp(physics_world_timestamp);
    }
    if(physics_field)
    {
        new_world.mutableField() =
                physics_field->getFieldWithTimestamp(physics_world_timestamp);
    }

    new_world.mutableFriendlyTeam().clearAllRobots();
    std::vector<Robot> friendly_robots;
    for(const auto& robot : friendly_physics_robots) {
        friendly_robots.emplace_back(robot->getRobotWithTimestamp(physics_world_timestamp));
    }
    new_world.mutableFriendlyTeam().updateRobots(friendly_robots);

    new_world.mutableEnemyTeam().clearAllRobots();
    std::vector<Robot> enemy_robots;
    for(const auto& robot : enemy_physics_robots) {
        enemy_robots.emplace_back(robot->getRobotWithTimestamp(physics_world_timestamp));
    }
    new_world.mutableEnemyTeam().updateRobots(enemy_robots);

    return new_world;
}

std::vector<std::weak_ptr<PhysicsRobot>> PhysicsSimulator::getFriendlyPhysicsRobots() const {
    std::vector<std::weak_ptr<PhysicsRobot>> robots;
    for(const auto& friendly_physics_robot : friendly_physics_robots) {
        robots.emplace_back(friendly_physics_robot);
    }
    return robots;
}

void PhysicsSimulator::setWorld(const World& world)
{
    physics_field = std::make_unique<PhysicsField>(physics_world, world.field());
    physics_ball  = std::make_unique<PhysicsBall>(physics_world, world.ball());
    friendly_physics_robots.clear();
    for(const auto& friendly_robot : world.friendlyTeam().getAllRobots()) {
        friendly_physics_robots.emplace_back(std::make_shared<PhysicsRobot>(physics_world, friendly_robot));
    }
    enemy_physics_robots.clear();
    for(const auto& enemy_robot : world.enemyTeam().getAllRobots()) {
        enemy_physics_robots.emplace_back(std::make_shared<PhysicsRobot>(physics_world, enemy_robot));
    }
    physics_world_timestamp = world.getMostRecentTimestamp();
}
