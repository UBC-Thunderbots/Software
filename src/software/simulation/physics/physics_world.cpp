#include "software/simulation/physics/physics_world.h"

#include <limits>

#include "shared/constants.h"
#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"

PhysicsWorld::PhysicsWorld(const Field& field,
                           std::shared_ptr<const SimulatorConfig> simulator_config)
    : b2_world(std::make_shared<b2World>(b2Vec2{0, 0})),
      current_timestamp(Timestamp::fromSeconds(0)),
      contact_listener(std::make_unique<SimulationContactListener>()),
      physics_field(b2_world, field),
      physics_ball(nullptr),
      simulator_config(simulator_config)
{
    b2_world->SetContactListener(contact_listener.get());
}

const Field PhysicsWorld::getField() const
{
    return physics_field.getField();
}

const std::optional<BallState> PhysicsWorld::getBallState() const
{
    return physics_ball ? std::make_optional(physics_ball->getBallState()) : std::nullopt;
}

const std::vector<RobotStateWithId> PhysicsWorld::getRobotStates(
    const TeamColour colour) const
{
    std::vector<std::shared_ptr<PhysicsRobot>> physics_robots;
    switch (colour)
    {
        case TeamColour::BLUE:
            physics_robots = blue_physics_robots;
            break;
        case TeamColour::YELLOW:
            physics_robots = yellow_physics_robots;
            break;
    }

    std::vector<RobotStateWithId> robot_states;
    for (const auto& robot : physics_robots)
    {
        if (!robot)
        {
            LOG(FATAL) << "Encountered a nullptr to a " << toString(colour)
                       << " physics robot in the physics world";
        }

        auto state_with_id = RobotStateWithId{.id          = robot->getRobotId(),
                                              .robot_state = robot->getRobotState()};
        robot_states.emplace_back(state_with_id);
    }

    return robot_states;
}

const std::vector<RobotStateWithId> PhysicsWorld::getYellowRobotStates() const
{
    return getRobotStates(TeamColour::YELLOW);
}

const std::vector<RobotStateWithId> PhysicsWorld::getBlueRobotStates() const
{
    return getRobotStates(TeamColour::BLUE);
}

const Timestamp PhysicsWorld::getTimestamp() const
{
    return current_timestamp;
}

void PhysicsWorld::setBallState(const BallState& ball_state)
{
    physics_ball = std::make_shared<PhysicsBall>(b2_world, ball_state, BALL_MASS_KG,
                                                 simulator_config);
}

void PhysicsWorld::removeBall()
{
    physics_ball.reset();
}

void PhysicsWorld::addYellowRobots(const std::vector<RobotStateWithId>& robots)
{
    for (const auto& state_with_id : robots)
    {
        if (isRobotIdAvailable(state_with_id.id, TeamColour::YELLOW))
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
        if (isRobotIdAvailable(state_with_id.id, TeamColour::BLUE))
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

RobotId PhysicsWorld::getAvailableRobotId(TeamColour colour) const
{
    for (RobotId i = 0; i < std::numeric_limits<RobotId>::max(); i++)
    {
        if (isRobotIdAvailable(i, colour))
        {
            return i;
        }
    }

    if (isRobotIdAvailable(std::numeric_limits<RobotId>::max(), colour))
    {
        return std::numeric_limits<RobotId>::max();
    }

    LOG(FATAL) << "Out of available " << toString(colour)
               << " robot IDs in the physics world";

    return std::numeric_limits<RobotId>::max();
}

RobotId PhysicsWorld::getAvailableYellowRobotId() const
{
    return getAvailableRobotId(TeamColour::YELLOW);
}

RobotId PhysicsWorld::getAvailableBlueRobotId() const
{
    return getAvailableRobotId(TeamColour::BLUE);
}

bool PhysicsWorld::isRobotIdAvailable(RobotId id, TeamColour colour) const
{
    std::vector<std::shared_ptr<PhysicsRobot>> physics_robots;
    switch (colour)
    {
        case TeamColour::BLUE:
            physics_robots = blue_physics_robots;
            break;
        case TeamColour ::YELLOW:
            physics_robots = yellow_physics_robots;
            break;
    }

    bool id_available = true;
    for (const auto& robot : physics_robots)
    {
        if (!robot)
        {
            LOG(FATAL) << "Encountered a nullptr to a " << toString(colour)
                       << " physics robot in the physics world";
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
    if (physics_ball)
    {
        physics_ball->updateIsInFlight();
        if (!physics_ball->isInFlight())
        {
            physics_ball->applyBallFrictionModel(time_step);
        }
    }
    b2_world->Step(static_cast<float>(time_step.toSeconds()), velocity_iterations,
                   position_iterations);

    for (const auto& physics_robots : {yellow_physics_robots, blue_physics_robots})
    {
        for (auto& robot : physics_robots)
        {
            robot->runPostPhysicsStep();
        }
    }

    current_timestamp = current_timestamp + time_step;
}

std::vector<std::weak_ptr<PhysicsRobot>> PhysicsWorld::getYellowPhysicsRobots() const
{
    std::vector<std::weak_ptr<PhysicsRobot>> robots;
    for (const auto& yellow_physics_robot : yellow_physics_robots)
    {
        if (!yellow_physics_robot)
        {
            LOG(FATAL)
                << "Encountered a nullptr to a yellow physics robot in the physics world";
        }

        robots.emplace_back(yellow_physics_robot);
    }
    return robots;
}

std::vector<std::weak_ptr<PhysicsRobot>> PhysicsWorld::getBluePhysicsRobots() const
{
    std::vector<std::weak_ptr<PhysicsRobot>> robots;
    for (const auto& blue_physics_robot : blue_physics_robots)
    {
        if (!blue_physics_robot)
        {
            LOG(FATAL)
                << "Encountered a nullptr to a blue physics robot in the physics world";
        }

        robots.emplace_back(blue_physics_robot);
    }
    return robots;
}

std::weak_ptr<PhysicsBall> PhysicsWorld::getPhysicsBall() const
{
    return std::weak_ptr<PhysicsBall>(physics_ball);
}

std::weak_ptr<PhysicsRobot> PhysicsWorld::getRobotAtPosition(const Point& position)
{
    std::weak_ptr<PhysicsRobot> result;

    for (const auto& physics_robots : {yellow_physics_robots, blue_physics_robots})
    {
        for (const auto& robot : physics_robots)
        {
            if (distance(position, robot->position()) < ROBOT_MAX_RADIUS_METERS)
            {
                result = robot;
            }
        }
    }

    return result;
}

void PhysicsWorld::removeRobot(std::weak_ptr<PhysicsRobot> robot)
{
    if (auto r = robot.lock())
    {
        yellow_physics_robots.erase(
            std::remove(yellow_physics_robots.begin(), yellow_physics_robots.end(), r),
            yellow_physics_robots.end());
        blue_physics_robots.erase(
            std::remove(blue_physics_robots.begin(), blue_physics_robots.end(), r),
            blue_physics_robots.end());
    }
}
