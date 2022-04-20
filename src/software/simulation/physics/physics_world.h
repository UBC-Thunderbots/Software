#pragma once

#include <Box2D/Box2D.h>

#include "proto/parameters.pb.h"
#include "software/multithreading/thread_safe_buffer.hpp"
#include "software/simulation/physics/physics_ball.h"
#include "software/simulation/physics/physics_field.h"
#include "software/simulation/physics/physics_robot.h"
#include "software/simulation/physics/simulation_contact_listener.h"
#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/ball_state.h"
#include "software/world/robot_state.h"
#include "software/world/world.h"

/**
 * This class represents a World in a Box2D physics simulation. It provides a convenient
 * way for us to abstract and hold a lot of the world's contents. It's also used to
 * convert to our own World class when needed.
 */
class PhysicsWorld
{
   public:
    /**
     * Creates a new PhysicsWorld that will contain no robots and no ball.
     *
     * @param field The initial state of the field
     * @param robot_constants The robot constants
     * @param wheel_constants The wheel constants
     * @param simulator_config The config to fetch parameters from
     */
    explicit PhysicsWorld(const Field& field, const RobotConstants_t& robot_constants,
                          const WheelConstants& wheel_constants,
                          TbotsProto::SimulatorConfig simulator_config);
    PhysicsWorld() = delete;

    // Delete the copy and assignment operators because copying this class causes
    // issues with the b2World and how it tracks bodies in the world, because as objects
    // are copied and destroyed, they will destroy the bodies in the b2World as well
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;
    PhysicsWorld(const PhysicsWorld&)            = delete;

    /**
     * Returns the field in the physics world
     *
     * @return the field in the physics world
     */
    const Field getField() const;

    /**
     * Returns the state of the ball in the physics world, if it exists
     *
     * @return the state of the ball in the physics world, if it exists
     */
    const std::optional<BallState> getBallState() const;

    /**
     * Returns the states and IDs of all yellow robots in the physics world.
     *
     * There is no guarantee as to what order the robots are returned in.
     *
     * @return the states and IDs of all yellow robots in the physics world
     */
    const std::vector<RobotStateWithId> getYellowRobotStates() const;

    /**
     * Returns the states and IDs of all blue robots in the physics world.
     *
     * There is no guarantee as to what order the robots are returned in.
     *
     * @return the states and IDs of all blue robots in the physics world
     */
    const std::vector<RobotStateWithId> getBlueRobotStates() const;

    /**
     * Returns the current timestamp of the physics world
     *
     * @return the current timestamp of the physics world
     */
    const Timestamp getTimestamp() const;

    /**
     * Sets the state of the ball in the physics world. No more than 1 ball may exist
     * in the physics world at a time. If there is no ball in the physics world, a ball
     * is added with the given state. If a ball already exists, it's state is set to the
     * given state.
     *
     * @param ball_state The new ball state
     */
    void setBallState(const BallState& ball_state);

    /**
     * Adds robots to the yellow team with the given initial states.
     *
     * @pre The robot IDs must not be duplicated and must not match the ID
     * of any robot already on the yellow team.
     *
     * @throws runtime_error if any of the given robot ids are duplicated, or a
     * yellow robot already exists with the ID
     *
     * @param robots the robots to add
     */
    void addYellowRobots(const std::vector<RobotStateWithId>& robots);

    /**
     * Adds robots to the yellow team with the given initial states.
     *
     * @pre The robot IDs must not be duplicated and must not match the ID
     * of any robot already on the blue team.
     *
     * @throws runtime_error if any of the given robot ids are duplicated, or a
     * yellow robot already exists with the ID
     *
     * @param robots the robots to add
     */
    void addBlueRobots(const std::vector<RobotStateWithId>& robots);

    /**
     * Returns the lowest available robot ID that is not already in use by a yellow robot.
     *
     * @return the lowest available robot ID that is not already in use by a yellow robot.
     */
    RobotId getAvailableYellowRobotId() const;

    /**
     * Returns the lowest available robot ID that is not already in use by a blue robot.
     *
     * @return the lowest available robot ID that is not already in use by a blue robot.
     */
    RobotId getAvailableBlueRobotId() const;

    /**
     * Advances the physics simulation by the given time step
     *
     * @param time_step how much to advance the world physics by
     */
    void stepSimulation(const Duration& time_step);

    /**
     * Returns the yellow PhysicsRobots currently in the world
     *
     * @return the yellow PhysicsRobots in the world
     */
    std::vector<std::weak_ptr<PhysicsRobot>> getYellowPhysicsRobots() const;

    /**
     * Returns the blue PhysicsRobots currently in the world
     *
     * @return the blue PhysicsRobots in the world
     */
    std::vector<std::weak_ptr<PhysicsRobot>> getBluePhysicsRobots() const;

    /**
     * Returns the PhysicsBall currently in the world
     *
     * @return the PhysicsBall in the world
     */
    std::weak_ptr<PhysicsBall> getPhysicsBall() const;

    /**
     * Returns the PhysicsRobot at the given position. This function accounts
     * for robot radius, so a robot will be returned if the given position is
     * within the robot's radius from its position.
     *
     * @param position The position at which to check for a robot
     *
     * @return a weak_ptr to the PhysicsRobot at the given position if one exists,
     * otherwise returns an empty pointer
     */
    std::weak_ptr<PhysicsRobot> getRobotAtPosition(const Point& position);

    /**
     * Removes the given PhysicsRobot from the PhysicsWorld, if it exists.
     *
     * @param robot The robot to be removed
     */
    void removeRobot(std::weak_ptr<PhysicsRobot> robot);

   private:
    /**
     * Returns the states and IDs of all robots of the specified colour.
     * There is no guarantee as to what order the robots are returned in.
     *
     * @param colour The colour of robot to return
     *
     * @return the states and IDs of all robots of the specified colour
     */
    const std::vector<RobotStateWithId> getRobotStates(TeamColour colour) const;

    /**
     * Returns the lowest available robot ID that is not already in use
     * by a robot of the specified colour
     *
     * @param colour The team colour
     *
     * @return the lowest available robot ID that is not already in use
     * by a robot of the specified colour
     */
    RobotId getAvailableRobotId(TeamColour colour) const;

    /**
     * Returns true if the given id is not already in use by the specified team
     *
     * @param id The id to check
     * @param colour the colour of the team to check against
     *
     * @return true if the given id is not already in use by the speciried team,
     * and false otherwise
     */
    bool isRobotIdAvailable(RobotId id, TeamColour colour) const;

    // Note: we declare the b2World first so it is destroyed last. If it is destroyed
    // before the physics objects, segfaults will occur due to pointers internal to Box2D
    // https://stackoverflow.com/questions/2254263/order-of-member-constructor-and-destructor-calls
    std::shared_ptr<b2World> b2_world;
    // The timestamp of the simulated physics world
    Timestamp current_timestamp;

    // 3 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    const unsigned int velocity_iterations = 8;
    const unsigned int position_iterations = 3;

    std::unique_ptr<SimulationContactListener> contact_listener;

    PhysicsField physics_field;
    std::shared_ptr<PhysicsBall> physics_ball;
    std::vector<std::shared_ptr<PhysicsRobot>> yellow_physics_robots;
    std::vector<std::shared_ptr<PhysicsRobot>> blue_physics_robots;
    RobotConstants_t robot_constants;
    WheelConstants_t wheel_constants;
    TbotsProto::SimulatorConfig simulator_config;
};
