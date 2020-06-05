#include "software/simulation/physics/physics_world.h"

#include <limits>

#include "shared/constants.h"
#include "software/logger/logger.h"

PhysicsWorld::PhysicsWorld(const Field& field)
    : b2_world(std::make_shared<b2World>(b2Vec2{0, 0})),
      current_timestamp(Timestamp::fromSeconds(0)),
      contact_listener(std::make_unique<SimulationContactListener>()),
      physics_field(std::make_shared<PhysicsField>(b2_world, field)),
      physics_ball(nullptr)
{
    b2_world->SetContactListener(contact_listener.get());
}

const Field PhysicsWorld::getField() const
{
    return physics_field->getField();
}

const std::optional<BallState> PhysicsWorld::getBallState() const
{
    return physics_ball ? std::make_optional(physics_ball->getBallState()) : std::nullopt;
}

const std::vector<RobotStateWithId> PhysicsWorld::getYellowRobotStates()
    const
{
    std::vector<RobotStateWithId> robot_states;
    for (const auto& robot : yellow_physics_robots)
    {
        auto state_with_id = RobotStateWithId{
            .id = robot->getRobotId(), .robot_state = robot->getRobotState()};
        robot_states.emplace_back(state_with_id);
    }

    return robot_states;
}

const std::vector<RobotStateWithId> PhysicsWorld::getBlueRobotStates() const
{
    std::vector<RobotStateWithId> robot_states;
    for (const auto& robot : blue_physics_robots)
    {
        auto state_with_id = RobotStateWithId{
            .id = robot->getRobotId(), .robot_state = robot->getRobotState()};
        robot_states.emplace_back(state_with_id);
    }

    return robot_states;
}

const Timestamp PhysicsWorld::getTimestamp() const
{
    return current_timestamp;
}

void PhysicsWorld::setField(const Field& field)
{
    if (field.isValid())
    {
        physics_field = std::make_shared<PhysicsField>(b2_world, field);
    }
}

void PhysicsWorld::setBallState(const BallState& ball_state)
{
    physics_ball = std::make_shared<PhysicsBall>(b2_world, ball_state, BALL_MASS_KG,
                                                 acceleration_due_to_gravity);
}

void PhysicsWorld::removeBall()
{
    physics_ball.reset();
}

void PhysicsWorld::addYellowRobots(const std::vector<RobotStateWithId>& robots)
{
    for (const auto& state_with_id : robots)
    {
        if (isYellowRobotIdAvailable(state_with_id.id))
        {
            yellow_physics_robots.emplace_back(std::make_shared<PhysicsRobot>(
                state_with_id.id, b2_world, state_with_id.robot_state,
                ROBOT_WITH_BATTERY_MASS_KG));
        }
        else
        {
            std::stringstream ss;
            ss << "Yellow robot with id " << state_with_id.id
               << " already exists in the physics world" << std::endl;
            throw std::runtime_error(ss.str());
        }
    }
}

void PhysicsWorld::addBlueRobots(const std::vector<RobotStateWithId>& robots)
{
    for (const auto& state_with_id : robots)
    {
        if (isBlueRobotIdAvailable(state_with_id.id))
        {
            blue_physics_robots.emplace_back(std::make_shared<PhysicsRobot>(
                state_with_id.id, b2_world, state_with_id.robot_state,
                ROBOT_WITH_BATTERY_MASS_KG));
        }
        else
        {
            std::stringstream ss;
            ss << "Blue robot with id " << state_with_id.id
               << " already exists in the physics world" << std::endl;
            throw std::runtime_error(ss.str());
        }
    }
}

unsigned int PhysicsWorld::getAvailableYellowRobotId() const
{
    for (unsigned int i = 0; i < std::numeric_limits<unsigned int>::max(); i++)
    {
        if (isYellowRobotIdAvailable(i))
        {
            return i;
        }
    }

    if (isYellowRobotIdAvailable(std::numeric_limits<unsigned int>::max()))
    {
        return std::numeric_limits<unsigned int>::max();
    }

    LOG(FATAL) << "Out of available yellow robot IDs in the physics world";
}

bool PhysicsWorld::isYellowRobotIdAvailable(unsigned int id) const
{
    bool id_available = true;
    for (const auto& robot : yellow_physics_robots)
    {
        if (!robot)
        {
            LOG(FATAL) << "Encountered a nullptr to a yellow physics robot in the physics world";
        }

        if (id == robot->getRobotId())
        {
            id_available = false;
            break;
        }
    }

    return id_available;
}

unsigned int PhysicsWorld::getAvailableBlueRobotId() const
{
    for (unsigned int i = 0; i < std::numeric_limits<unsigned int>::max(); i++)
    {
        if (isBlueRobotIdAvailable(i))
        {
            return i;
        }
    }

    if (isBlueRobotIdAvailable(std::numeric_limits<unsigned int>::max()))
    {
        return std::numeric_limits<unsigned int>::max();
    }

    LOG(FATAL) << "Out of available blue robot IDs in the physics world";
}

bool PhysicsWorld::isBlueRobotIdAvailable(unsigned int id) const
{
    bool id_available = true;
    for (const auto& robot : blue_physics_robots)
    {
        if (!robot)
        {
            LOG(FATAL) << "Encountered a nullptr to a blue physics robot in the physics world";
        }

        if (id == robot->getRobotId())
        {
            id_available = false;
            break;
        }
    }

    return id_available;
}

void PhysicsWorld::stepSimulation(const Duration& time_step)
{
    b2_world->Step(time_step.getSeconds(), velocity_iterations, position_iterations);
    current_timestamp = current_timestamp + time_step;
}

std::vector<std::weak_ptr<PhysicsRobot>> PhysicsWorld::getFriendlyPhysicsRobots() const
{
    std::vector<std::weak_ptr<PhysicsRobot>> robots;
    for (const auto& friendly_physics_robot : yellow_physics_robots)
    {
        robots.emplace_back(friendly_physics_robot);
    }
    return robots;
}

std::weak_ptr<PhysicsBall> PhysicsWorld::getPhysicsBall() const
{
    return std::weak_ptr<PhysicsBall>(physics_ball);
}
