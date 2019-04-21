#include "network_input/filter/robot_filter.h"

RobotFilter::RobotFilter(unsigned int id, Duration expiry_buffer_duration)
    : robot_id(id), expiry_buffer_duration(expiry_buffer_duration)
{
}

std::optional<Robot> RobotFilter::getFilteredData(
    std::optional<Robot> current_robot_state,
    const std::vector<SSLRobotDetection> &new_robot_data)
{
    // if current robot state is not initialized, initialized it
    if (!current_robot_state.has_value())
    {
        current_robot_state.emplace(Robot(this->getRobotId(),Point(),Vector(),Angle(),AngularVelocity(),Timestamp()));
    }

    int data_num                = 0;
    Timestamp largest_timestamp = Timestamp();
    FilteredRobotData filtered_data;

    filtered_data.id               = this->getRobotId();
    filtered_data.position         = Point();
    filtered_data.velocity         = Vector();
    filtered_data.orientation      = Angle();
    filtered_data.angular_velocity = AngularVelocity();
    filtered_data.timestamp        = Timestamp();

    for (const SSLRobotDetection &robot_data : new_robot_data)
    {
        // add up all data points for this robot and then average it
        if (robot_data.id == this->getRobotId() &&
            robot_data.timestamp >= current_robot_state.value().lastUpdateTimestamp())
        {
            filtered_data.position += robot_data.position;
            filtered_data.orientation += robot_data.orientation;
            filtered_data.timestamp.fromMilliseconds(
                filtered_data.timestamp.getMilliseconds() +
                robot_data.timestamp.getMilliseconds());
            data_num++;
        }

        // to get the largest timestamp of all data points in case there is no data for
        // this robot id
        if (largest_timestamp.getMilliseconds() < robot_data.timestamp.getMilliseconds())
        {
            largest_timestamp = robot_data.timestamp;
        }
    }

    if (data_num == 0)
    {
        // if there is no data after the time of expiry_buffer_duration than previously
        // recorded robot state, return null. Otherwise remain the same state
        if (largest_timestamp.getMilliseconds() >
            this->expiry_buffer_duration.getMilliseconds() +
                current_robot_state.value().lastUpdateTimestamp().getMilliseconds())
        {
            return std::nullopt;
        }
        else
        {
            return std::make_optional(current_robot_state.value());
        }
    }
    else
    {
        // update data by returning filtered robot data
        filtered_data.position /= data_num;
        filtered_data.orientation /= data_num;

        filtered_data.timestamp.fromMilliseconds(
            filtered_data.timestamp.getMilliseconds() / data_num);

        // position difference/ time difference
        filtered_data.velocity =
            (filtered_data.position - current_robot_state.value().position()) /
            (filtered_data.timestamp.getSeconds() -
             current_robot_state.value().lastUpdateTimestamp().getSeconds());

        // orientation difference / time difference
        filtered_data.angular_velocity =
            (filtered_data.orientation - current_robot_state.value().orientation()) /
            (filtered_data.timestamp.getSeconds() -
             current_robot_state.value().lastUpdateTimestamp().getSeconds());

        return std::make_optional(Robot(this->getRobotId(), filtered_data.position,
                                        filtered_data.velocity, filtered_data.orientation,
                                        filtered_data.angular_velocity,
                                        filtered_data.timestamp));
    }
}

unsigned int RobotFilter::getRobotId() const
{
    return robot_id;
}
