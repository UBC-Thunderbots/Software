#pragma once

#include <optional>
#include <vector>

#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/time/timestamp.h"
#include "software/world/robot.h"

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
    explicit RobotFilter(RobotDetection current_robot_state,
                         Duration expiry_buffer_duration);

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
        const std::vector<RobotDetection>& new_robot_data);

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
