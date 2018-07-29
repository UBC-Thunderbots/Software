#pragma once

#include "geom/angle.h"
#include "geom/point.h"

/**
 * A lightweight datatype used to input new data into the filter.
 * We do this rather than taking the SSL_Detection data directly
 * so we can make this module more generic and abstract away
 * the protobuf for testing
 */
typedef struct
{
    unsigned int id;
    Point position;
    Angle orientation;
    double confidence;
} SSLRobotData;

class RobotFilter
{
   public:
    /**
     * Creates a new robot filter
     *
     * @param id the id of the robot to filter
     */
    explicit RobotFilter(unsigned int id);

    /**
     * Updates the filter given a new set of data
     */
    // TODO: Could this accept a vector? Are we likely to get multiple robot detections at
    // once?
    void update(const SSLRobotData &new_robot_data);

    /**
     * Returns the filtered position of the robot
     *
     * @return the filtered position of the robot
     */
    Point getRobotPosition();

    /**
     * Returns the filtered velocity of the robot
     *
     * @return the filtered velocity of the robot
     */
    Point getRobotVelocity();

    /**
     * Gets the filtered orientation of the robot
     *
     * @return the filtered orientation of the robot
     */
    Angle getRobotOrientation();

    /**
     * Gets the id of the robot
     *
     * @return the id of the robot
     */
    unsigned int getRobotId();

   private:
    unsigned int robot_id;
    Point current_robot_position;
    Point current_robot_velocity;
    Angle current_robot_orientation;
};
