#pragma once

extern "C"
{
#include "firmware/app/world/firmware_ball.h"
}
#include "software/simulation/simulator_ball.h"

/**
 * Because the FirmwareBall_t struct is defined in the .c file (rather than the .h file),
 * C++ considers it an incomplete type and is unable to use it with smart pointers
 * because it doesn't know the size of the object. Therefore we need to create our own
 * "Deleter" class we can provide to the smart pointers to handle that instead.
 *
 * See https://en.cppreference.com/w/cpp/memory/unique_ptr/unique_ptr for more info and
 * examples
 */
struct FirmwareBallDeleter
{
    void operator()(FirmwareBall_t* firmware_ball) const
    {
        app_firmware_ball_destroy(firmware_ball);
    };
};

/**
 * This class acts as a wrapper around a SimulatorBall so that the SimulatorBall
 * can provide the interface of a FirmwareBall
 *
 * Because our firmware structs rely on C-style function pointers, we
 * cannot use C++ constructs like std::function to provide to the structs.
 * Therefore we use this class's static functions to provide the function
 * pointers, since static functions work as C-style function pointers
 * (because there is no instance associated with them).
 *
 * Whenever a caller needs to get a firmware struct or perform an operation, they
 * need to set which ball they want to control so that all the static functions
 * that have been provided to the firmware struct operate on the correct
 * instantiated object. This is our workaround to maintain and simulate multiple
 * "instances" of firmware at once.
 */
class SimulatorBallSingleton
{
   public:
    /**
     * Sets the SimulatorBall being controlled by this class
     *
     * @param ball The SimulatorBall to control with this class. Must not be null
     */
    static void setSimulatorBall(std::shared_ptr<SimulatorBall> ball);

    /**
     * Creates a FirmwareBall_t with functions bound to the static functions in this
     * class. Only one FirmwareBall_t needs to be created to control all balls, since
     * calling setSimulatorBall will simply change the implementations of the bound
     * functions to act as if the new ball was being controlled.
     *
     * @return a FirmwareBall_t corresponding to the whatever SimulatorBall this
     * Singleton is controlling
     */
    static std::unique_ptr<FirmwareBall_t, FirmwareBallDeleter> createFirmwareBall();

   private:
    /**
     * Returns the x-position of the current physics ball
     *
     * @return the x-position of the current physics ball
     */
    static float getBallPositionX();

    /**
     * Returns the y-position of the current physics ball
     *
     * @return the y-position of the current physics ball
     */
    static float getBallPositionY();

    /**
     * Returns the x-velocity of the current physics ball
     *
     * @return the x-velocity of the current physics ball
     */
    static float getBallVelocityX();

    /**
     * Returns the y-velocity of the current physics ball
     *
     * @return the y-velocity of the current physics ball
     */
    static float getBallVelocityY();

    /**
     * Helper functions that check if the pointer to the simulator_ball is valid before
     * calling the given function. If the simulator_ball is invalid, a warning is logged
     * and a default value is returned.
     *
     * @param func The function to perform on the simulator ball
     */
    static float checkValidAndReturnFloat(
        std::function<float(std::shared_ptr<SimulatorBall>)> func);

    // The simulator ball being controlled by this class
    static std::shared_ptr<SimulatorBall> simulator_ball;
};
