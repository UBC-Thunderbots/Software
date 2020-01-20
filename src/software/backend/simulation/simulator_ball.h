#pragma once

extern "C"
{
#include "app/world/firmware_ball.h"
}
#include "software/backend/simulation/physics/physics_ball.h"

/**
 * This class acts as a wrapper around a PhysicsBall so that the PhysicsBall
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
     * Sets the PhysicsBall being controlled by this class
     *
     * @param ball The PhysicsBall to control by this class. Must not be null
     */
    static void setPhysicsBall(std::weak_ptr<PhysicsBall> ball);

    /**
     * Creates a FirmwareBall corresponding to the current PhysicsBall
     *
     * @return a FirmwareBall corresponding to the current PhysicsBall
     */
    static FirmwareBall_t* createFirmwareBall();

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

    // The physics ball being controlled by this class
    static std::weak_ptr<PhysicsBall> physics_ball_weak_ptr;
};
