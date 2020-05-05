#include "software/simulation/physics/physics_world.h"

#include "shared/constants.h"

PhysicsWorld::PhysicsWorld(const World& world)
{
    b2Vec2 gravity(0, 0);
    b2_world          = std::make_shared<b2World>(gravity);
    current_timestamp = Timestamp::fromSeconds(0);

    contact_listener = std::make_unique<SimulationContactListener>();
    b2_world->SetContactListener(contact_listener.get());

    initWorld(world);
}

void PhysicsWorld::initWorld(const World& world)
{
    physics_field = std::make_shared<PhysicsField>(b2_world, world.field());
    physics_ball  = std::make_shared<PhysicsBall>(b2_world, world.ball(), BALL_MASS_KG,
                                                 acceleration_due_to_gravity);
    friendly_physics_robots.clear();
    for (const auto& friendly_robot : world.friendlyTeam().getAllRobots())
    {
        friendly_physics_robots.emplace_back(std::make_shared<PhysicsRobot>(
            b2_world, friendly_robot, ROBOT_WITH_BATTERY_MASS_KG));
    }
    enemy_physics_robots.clear();
    for (const auto& enemy_robot : world.enemyTeam().getAllRobots())
    {
        enemy_physics_robots.emplace_back(std::make_shared<PhysicsRobot>(
            b2_world, enemy_robot, ROBOT_WITH_BATTERY_MASS_KG));
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

void PhysicsWorld::stepSimulation(const Duration& time_step)
{
    b2_world->Step(time_step.getSeconds(), velocity_iterations, position_iterations);
    current_timestamp = current_timestamp + time_step;
}

std::vector<std::weak_ptr<PhysicsRobot>> PhysicsWorld::getFriendlyPhysicsRobots() const
{
    std::vector<std::weak_ptr<PhysicsRobot>> robots;
    for (const auto& friendly_physics_robot : friendly_physics_robots)
    {
        robots.emplace_back(friendly_physics_robot);
    }
    return robots;
}

std::weak_ptr<PhysicsBall> PhysicsWorld::getPhysicsBall() const
{
    return std::weak_ptr<PhysicsBall>(physics_ball);
}
