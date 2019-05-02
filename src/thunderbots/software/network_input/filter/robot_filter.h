#pragma once

#include <optional>
#include <vector>

#include "ai/world/robot.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "util/time/timestamp.h"

/**
 * A lightweight datatype used to input new data into the filter.
 * We do this rather than taking the SSL_Detection data directly
 * so we can make this module more generic and abstract away
 * the protobuf for testing
 */
struct SSLRobotDetection
{
    unsigned int id;
    Point position;
    Angle orientation;
    double confidence;
    Timestamp timestamp;
};

/**
 * A lightweight datatype used to pass filtered robot data
 */
typedef struct FilteredRobotData_t
{
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
     * @param current_robot_state the data of current state of the robot
     * @param expiry_buffer_duration the time when the robot is determined to be removed
     * from the field if data about the robot is not received before that time
     */
    explicit RobotFilter(Robot current_robot_state, Duration expiry_buffer_duration);

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
    std::optional<Robot> getFilteredData(
        const std::vector<SSLRobotDetection>& new_robot_data);

    /**
     * Returns the id of the Robot that this filter is filtering for
     *
     * @return the id of the Robot that this filter is filtering for
     */
    unsigned int getRobotId() const;

   private:
    Robot current_robot_state;
    Duration expiry_buffer_duration;
};
