#include "software/sensor_fusion/filter/robot_filter.h"

RobotFilter::RobotFilter(Robot current_robot_state, Duration expiry_buffer_duration)
    : current_robot_state(current_robot_state),
      expiry_buffer_duration(expiry_buffer_duration)
{
}

RobotFilter::RobotFilter(RobotDetection current_robot_state,
                         Duration expiry_buffer_duration)
    : current_robot_state(current_robot_state.id, current_robot_state.position,
                          Vector(0, 0), current_robot_state.orientation,
                          AngularVelocity::zero(), current_robot_state.timestamp),
      expiry_buffer_duration(expiry_buffer_duration)
{
}

std::optional<Robot> RobotFilter::getFilteredData(
    const std::vector<RobotDetection> &new_robot_data)
{
    int data_num               = 0;
    Timestamp latest_timestamp = Timestamp().fromSeconds(0);
    FilteredRobotData filtered_data{.id               = this->getRobotId(),
                                    .position         = Point(0, 0),
                                    .velocity         = Vector(0, 0),
                                    .orientation      = Angle::fromRadians(0),
                                    .angular_velocity = AngularVelocity::fromRadians(0),
                                    .timestamp        = Timestamp().fromSeconds(0)};

    for (const RobotDetection &robot_data : new_robot_data)
    {
        // add up all data points for this robot and then average it
        if (robot_data.id == this->getRobotId() &&
            robot_data.timestamp > this->current_robot_state.timestamp())
        {
            filtered_data.position =
                filtered_data.position + robot_data.position.toVector();
            filtered_data.orientation =
                filtered_data.orientation + robot_data.orientation;

            filtered_data.timestamp = filtered_data.timestamp.fromMilliseconds(
                filtered_data.timestamp.toMilliseconds() +
                robot_data.timestamp.toMilliseconds());
            data_num++;
        }

        // to get the latest timestamp of all data points in case there is no data for
        // this robot id
        if (latest_timestamp.toMilliseconds() < robot_data.timestamp.toMilliseconds())
        {
            latest_timestamp = robot_data.timestamp;
        }
    }

    if (data_num == 0)
    {
        // if there is no data the duration of expiry_buffer_duration after previously
        // recorded robot state, return null. Otherwise remain the same state
        if (latest_timestamp.toMilliseconds() >
            this->expiry_buffer_duration.toMilliseconds() +
                current_robot_state.timestamp().toMilliseconds())
        {
            return std::nullopt;
        }
        else
        {
            return std::make_optional(current_robot_state);
        }
    }
    else
    {
        // update data by returning filtered robot data
        filtered_data.position    = Point(filtered_data.position.toVector() / data_num);
        filtered_data.orientation = filtered_data.orientation / data_num;

        filtered_data.timestamp = filtered_data.timestamp.fromMilliseconds(
            filtered_data.timestamp.toMilliseconds() / data_num);

        // velocity = position difference / time difference
        filtered_data.velocity =
            (filtered_data.position - current_robot_state.position()) /
            (filtered_data.timestamp.toSeconds() -
             current_robot_state.timestamp().toSeconds());

//        if (filtered_data.velocity.length() <= 0.15) {
//            filtered_data.velocity = Vector(0, 0);
//        }

        // angular_velocity = orientation difference / time difference
        filtered_data.angular_velocity =
            (filtered_data.orientation - current_robot_state.orientation()).clamp() /
            (filtered_data.timestamp.toSeconds() -
             current_robot_state.timestamp().toSeconds());

        // update current_robot_state
        this->current_robot_state =
            Robot(this->getRobotId(), filtered_data.position, filtered_data.velocity,
                  filtered_data.orientation, filtered_data.angular_velocity,
                  filtered_data.timestamp);

        return std::make_optional(this->current_robot_state);
    }
}

unsigned int RobotFilter::getRobotId() const
{
    return this->current_robot_state.id();
}
