#ifndef AI_WORLD_ROBOT_H_
#define AI_WORLD_ROBOT_H_

#include "geom/angle.h"
#include "geom/point.h"
#include "thunderbots_msgs/Robot.h"

/**
 * Defines an SSL robot
 */
class Robot
{
   public:
    // The max radius of a robot in metres
    static constexpr double MAX_RADIUS = 0.09;

    /**
     * Creates a new robot given a pattern id
     */
    explicit Robot(unsigned int id);

    /**
     * Creates a new robot given a pattern id
     */
    explicit Robot(const thunderbots_msgs::Robot& robot_msg);

    /**
     * Updates the state of the robot.
     *
     * @param new_position the new position of the robot
     * @param new_velocity the new velocity of the robot
     * @param new_orientation the new orientation of the robot
     * @param new_angular_velocity the new angular velocity of the robot
     */
    void update(
        const Point& new_position, const Point& new_velocity,
        const Angle& new_orientation);

    /**
     * Returns the id of the robot
     *
     * @return the id of the robot
     */
    unsigned int id() const;

    /**
     * Get the predicted position of the robot at a time relative to the current time.
     * Using the default
     * of 0 will give the current position of the robot.
     *
     * @param time_delta the amount of time forwards to predict.
     *
     * @return the predicted position of the robot.
     */
    Point position(double time_delta = 0.0) const;

    /**
     * Get the predicted velocity of the robot at a time relative to the current time.
     * Using the default
     * of 0 will give the current velocity of the robot.
     *
     * @param time_delta The amount of time in seconds forwards to predict.
     *
     * @return the predicted velocity of the robot.
     */
    Point velocity(double time_delta = 0.0) const;

    /**
     * Get the predicted orientation of the robot at a time relative to the current time.
     * Using the default
     * of 0 will give the current orientation of the robot.
     *
     * @param time_delta the amount of time in seconds forwards to predict.
     *
     * @return the predicted orientation of the robot.
     */
    Angle orientation(double time_delta = 0.0) const;

    /**
     * Get the predicted angular velocity of the robot at a time relative to the current
     * time.
     * Using the default of 0 will give the current orientation of the robot.
     *
     * @param time_delta the amount of time in seconds forwards the predict.
     *
     * @return the predicted orientation of the robot.
     */
    Angle angularVelocity(double time_delta = 0.0) const;

   private:
    const unsigned int id_;
    Point position_;
    Point velocity_;
    Angle orientation_;
    Angle angularVelocity_;
};

#endif  // AI_WORLD_ROBOT_H_
