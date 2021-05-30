#pragma once

#include <functional>
#include <memory>

#include "software/logger/logger.h"
#include "software/simulation/firmware_object_deleter.h"
#include "software/simulation/simulator_ball.h"

extern "C"
{
#include "firmware/app/world/firmware_ball.h"
}
#include "software/simulation/simulator_ball.h"
#include "software/world/field.h"

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
     * Sets the SimulatorBall being controlled by this class.
     *
     * The attributes of the ball, such as position, will be given for the POV of
     * a robot defending the specified field side. Eg. If the ball is at (-1, 2)
     * in real-world coordinates, it's position will be reported as (-1, 2) if
     * the negative field side is specified. On the other hand if the positive
     * field side is specified, the ball's position will be reported as (1, -2).
     *
     * This different behaviour for either field side exists because our firmware
     * expects its knowledge of the world to match our coordinate convention, which is
     * relative to the side of the field the robot is defending. See
     * https://github.com/UBC-Thunderbots/Software/blob/master/docs/software-architecture-and-design.md#coordinates
     * for more information about our coordinate conventions. Because we can't actually
     * change the positions and dynamics of the underlying physics objects, we use this
     * class to enforce this convention for the firmware.
     *
     * @param ball The SimulatorBall to control with this class. Must not be null
     * @param field_side The side of the field being defended by the robots using
     * this class
     */
    static void setSimulatorBall(std::shared_ptr<SimulatorBall> ball,
                                 FieldSide field_side);

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
     * @tparam RET_VAL the return value of the function to execute
     * @param func The function to perform on the simulator ball
     */
    template <class RET_VAL>
    static RET_VAL checkValidAndExecute(
        std::function<RET_VAL(std::shared_ptr<SimulatorBall>)> func)
    {
        if (simulator_ball)
        {
            return func(simulator_ball);
        }
        LOG(WARNING)
            << "SimulatorBallSingleton called without setting the SimulatorBall first"
            << std::endl;
        return static_cast<RET_VAL>(0);
    }

    /**
     * A helper function that will negate the given value if needed
     * in order for it to match our coordinate convention for the
     * current value of field_side
     *
     * @param value The value to invert
     *
     * @throws std::invalid_argument if there is an unhandled value of FieldSide
     *
     * @return value if field_side_ is NEG_X, and -value if field_side_
     * is POS_X
     */
    static float invertValueToMatchFieldSide(double value);

    // The simulator ball being controlled by this class
    static std::shared_ptr<SimulatorBall> simulator_ball;
    static FieldSide field_side_;
};
