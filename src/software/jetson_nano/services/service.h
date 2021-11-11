/**
 * A service defines an interface with hardware/software on the robot.
 */
class Service
{
   public:
    /**
     * Start the service.
     */
    virtual void start() = 0;

    /**
     * Stop the service.
     */
    virtual void stop() = 0;
};
