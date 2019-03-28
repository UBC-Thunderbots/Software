#include "network_input/filter/robot_filter.h"

RobotFilter::RobotFilter(unsigned int id) : robot_id(id) {}

FilteredRobotData RobotFilter::getFilteredData(
    const std::vector<SSLRobotDetection> &new_robot_data)
{
    FilteredRobotData filtered_data;

    //std::vector<SSLRobotDetection> this_robot_data;

    int data_num =0;

    filtered_data.id               = 0;
    filtered_data.position         = Point();
    filtered_data.velocity         = Vector(;
    filtered_data.orientation      = Angle::zero();
    filtered_data.angular_velocity = AngularVelocity::zero();
    filtered_data.timestamp        = Timestamp::fromSeconds(0);

    for(const SSLRobotDetection &robot_data: new_robot_data)
    {
        //add all data points up and then average it
        if(robot_data.id==this->getRobotId())
        {

            filtered_data.position         += robot_data.position;
            filtered_data.orientation      += robot_data.orientation;
            filtered_data.timestamp =  robot_data.timestamp;
            data_num++;
        }
    }



    //if no new data and timestamp difference is long enough, return filtered data as a indication that
    //the robot is off the field? if no new data and timestamp difference is small enough, then return current robot position
    if(data_num==0)
    {

    }
    else
    {
        filtered_data.position         /= data_num;
        filtered_data.orientation      /= data_num;

        return filtered_data;
    }

}

unsigned int RobotFilter::getRobotId() const
{
    return robot_id;
}
