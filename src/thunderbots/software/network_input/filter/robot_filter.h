#pragma once

#include <vector>

#include "geom/angle.h"
#include "geom/point.h"
#include "util/timestamp.h"

/**
 * A lightweight datatype used to input new data into the filter.
 * We do this rather than taking the SSL_Detection data directly
 * so we can make this module more generic and abstract away
 * the protobuf for testing
 */
typedef struct SSLRobotData
{
    /**
     * Creates a new SSLRobotData struct
     *
     * @param id The id of the robot
     * @param position The position of the robot
     * @param orientation The orientation of the robot
     * @param confidence The confidence (0.0 <= confidence <= 1.0) of this robot data
     * @param timestamp The timestamp of this data
     */
    SSLRobotData(unsigned int id, Point position, Angle orientation, double confidence,
                 Timestamp timestamp)
        : id(id),
          position(position),
          orientation(orientation),
          confidence(confidence),
          timestamp(timestamp)
    {
    }

    unsigned int id;
    Point position;
    Angle orientation;
    double confidence;
    Timestamp timestamp;
} SSLRobotData;

/**
 * A lightweight datatype used to pass filtered robot data
 */
typedef struct FilteredRobotData
{
    /**
     * Creates a new FilteredRobotData struct
     *
     * @param id The id of the robot
     * @param position The position of the robot
     * @param velocity The velocity of the robot
     * @param orientation The orientation of the robot
     * @param angular_velocity The angular velocity of the robot
     * @param timestamp The timestamp of the robot
     */
    FilteredRobotData(unsigned int id, Point position, Vector velocity, Angle orientation,
                      AngularVelocity angular_velocity, Timestamp timestamp)
        : id(id),
          position(position),
          velocity(velocity),
          angular_velocity(angular_velocity),
          timestamp(timestamp)
    {
    }

    unsigned int id;
    Point position;
    Vector velocity;
    Angle orientation;
    AngularVelocity angular_velocity;
    Timestamp timestamp;
} FilteredRobotData;

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
     * Updates the filter given a new set of data, and returns the most up to date
     * filtered data for the Robot.
     *
     * @param new_robot_data A list of SSLRobot detections containing new robot data.
     * The data does not all have to be for a particular Robot, the filter will only use
     * the new Robot data that matches the robot id the filter was constructed with.
     *
     * @return The filtered data for the robot
     */
    FilteredRobotData getFilteredData(const std::vector<SSLRobotData> &new_robot_data);

    /**
     * Returns the id of the Robot that this filter is filtering for
     *
     * @return the id of the Robot that this filter is filtering for
     */
    unsigned int getRobotId() const;

   private:
    unsigned int robot_id;
};
