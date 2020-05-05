#pragma once

#include "software/primitive/primitive.h"
#include "software/simulation/physics/physics_world.h"
#include "software/simulation/simulator_ball.h"
#include "software/simulation/simulator_robot.h"
#include "software/world/world.h"

/**
 * Because the FirmwareWorld_t struct is defined in the .c file (rather than the .h file),
 * C++ considers it an incomplete type and is unable to use it with smart pointers
 * because it doesn't know the size of the object. Therefore we need to create our own
 * "Deleter" class we can provide to the smart pointers to handle that instead.
 *
 * See https://en.cppreference.com/w/cpp/memory/unique_ptr/unique_ptr for more info and
 * examples
 */
struct FirmwareWorldDeleter
{
    void operator()(FirmwareWorld_t* firmware_world) const
    {
        FirmwareRobot_t* firmware_robot = app_firmware_world_getRobot(firmware_world);

        Wheel_t* firmware_robot_front_left_wheel =
            app_firmware_robot_getFrontLeftWheel(firmware_robot);
        app_wheel_destroy(firmware_robot_front_left_wheel);
        Wheel_t* firmware_robot_back_left_wheel =
            app_firmware_robot_getBackLeftWheel(firmware_robot);
        app_wheel_destroy(firmware_robot_back_left_wheel);
        Wheel_t* firmware_robot_back_right_wheel =
            app_firmware_robot_getBackRightWheel(firmware_robot);
        app_wheel_destroy(firmware_robot_back_right_wheel);
        Wheel_t* firmware_robot_front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot);
        app_wheel_destroy(firmware_robot_front_right_wheel);

        Chicker_t* firmware_robot_chicker = app_firmware_robot_getChicker(firmware_robot);
        app_chicker_destroy(firmware_robot_chicker);

        Dribbler_t* firmware_robot_dribbler =
            app_firmware_robot_getDribbler(firmware_robot);
        app_dribbler_destroy(firmware_robot_dribbler);

        app_firmware_robot_destroy(firmware_robot);

        FirmwareBall_t* firmware_ball = app_firmware_world_getBall(firmware_world);
        app_firmware_ball_destroy(firmware_ball);

        app_firmware_world_destroy(firmware_world);
    };
};

/**
 * The Simulator abstracts away the physics simulation of all objects in the world,
 * as well as the firmware simulation for the robots. This provides a simple interface
 * to setup, run, and query the current state of the simulation.
 */
class Simulator
{
   public:
    /**
     * Creates a new Simulator with the given world. The starting state of the simulation
     * will match the state of the given world
     *
     * @param world The world to initialize the simulation with
     */
    explicit Simulator(const World& world);
    Simulator() = delete;

    /**
     * Advances the simulation by the given time step. This will simulate
     * physics and primitives
     *
     * @param time_step how much to advance the simulation by
     */
    void stepSimulation(const Duration& time_step);

    /**
     * Sets the primitives being simulated by the robots in simulation
     *
     * @param primitives The primitives to simulate
     */
    void setPrimitives(ConstPrimitiveVectorPtr primitives);

    /**
     * Returns the current state of the world in the simulation
     *
     * @return the current state of the world in the simulation
     */
    World getWorld();

   private:
    /**
     * Returns the encoded primitive parameters for the given Primitive
     *
     * @param primitive The Primitive to get the parameters for
     *
     * @return The encoded primitive parameters for the given Primitive
     */
    primitive_params_t getPrimitiveParams(const std::unique_ptr<Primitive>& primitive);

    /**
     * Returns the primitive index for the given Primitive
     *
     * @param primitive The Primitive to get the index for
     *
     * @return The index for the given Primitive
     */
    unsigned int getPrimitiveIndex(const std::unique_ptr<Primitive>& primitive);

    PhysicsWorld physics_world;
    std::shared_ptr<SimulatorBall> simulator_ball;
    std::map<std::shared_ptr<SimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>
        simulator_robots;
};
