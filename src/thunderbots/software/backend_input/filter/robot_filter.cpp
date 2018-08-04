#include "robot_filter.h"
#include <ros/console.h>

RobotFilter::RobotFilter(unsigned int id)
    : current_robot_position(Point()),
      current_robot_velocity(Point()),
      current_robot_orientation(Angle::zero()),
      robot_id(id)
{
}

void RobotFilter::update(const SSLRobotData &new_robot_data)
{
    if (new_robot_data.id != robot_id)
    {
        ROS_DEBUG("Error: Different robot data given to robot filter");
    }

    Point previous_position   = current_robot_position;
    current_robot_position    = new_robot_data.position;
    current_robot_velocity    = current_robot_position - previous_position;
    current_robot_orientation = new_robot_data.orientation;
}

Point RobotFilter::getRobotPosition()
{
    return current_robot_position;
}

Point RobotFilter::getRobotVelocity()
{
    return current_robot_velocity;
}

Angle RobotFilter::getRobotOrientation()
{
    return current_robot_orientation;
}

unsigned int RobotFilter::getRobotId()
{
    return robot_id;
}
