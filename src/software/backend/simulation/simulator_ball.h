#include "app/world/firmware_ball.h"
#include "software/backend/simulation/physics/physics_ball.h"

/**
 * This class acts as a wrapper around a PhysicsBall so that the PhysicsBall
 * can provide the interface of a FirmwareBall
 */
class SimulatorBall {
public:
    /**
     * Sets the PhysicsBall being controlled by this class
     *
     * @param ball The PhysicsBall to control by this class. Must not be null
     */
    void setPhysicsBall(std::shared_ptr<PhysicsBall> ball);

    /**
     * Creates a FirmwareBall corresponding to the current PhysicsBall
     *
     * @return a FirmwareBall corresponding to the current PhysicsBall
     */
    static FirmwareBall_t* createFirmwareBall();

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

private:
    /**
     * Checks that the current physics_ball is a valid value
     *
     * @throws std::invalid_argument if the physics_ball is a nullptr
     */
    void checkPhysicsBallValid();

    static std::shared_ptr<PhysicsBall> physics_ball;
};