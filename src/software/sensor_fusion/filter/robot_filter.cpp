#include "software/sensor_fusion/filter/robot_filter.h"

RobotFilter::RobotFilter(Robot current_robot_state, Duration expiry_buffer_duration)
    : current_robot_state(current_robot_state),
      expiry_buffer_duration(expiry_buffer_duration)
{
}
// change the constructor initializations
RobotFilter::RobotFilter(RobotDetection current_robot_state,
                         Duration expiry_buffer_duration)
    : current_robot_state(current_robot_state.id, current_robot_state.position,
                          Vector(0, 0), current_robot_state.orientation,
                          AngularVelocity::zero(), current_robot_state.timestamp),
      expiry_buffer_duration(expiry_buffer_duration)
{
}

std::optional<Robot> RobotFilter::getFilteredData(
    const std::vector<RobotDetection>& new_robot_data, const Timestamp& capture_timestamp,
    const std::optional<RobotId> breakbeam_tripped_id)
{
    int data_num               = 0;
    Timestamp latest_timestamp = capture_timestamp;
    FilteredRobotData filtered_data{.id               = this->getRobotId(),
                                    .position         = Point(0, 0),
                                    .velocity         = Vector(0, 0),
                                    .orientation      = Angle::fromRadians(0),
                                    .orientation_cos  = 0.0,
                                    .orientation_sin  = 0.0,
                                    .angular_velocity = AngularVelocity::fromRadians(0),
                                    .timestamp        = Timestamp().fromSeconds(0)};

    for (const RobotDetection& robot_data : new_robot_data)
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

        // angular_velocity = orientation difference / time difference
        filtered_data.angular_velocity =
            (filtered_data.orientation - current_robot_state.orientation()).clamp() /
            (filtered_data.timestamp.toSeconds() -
             current_robot_state.timestamp().toSeconds());

        // find breakbeam_status
        bool breakbeam_tripped = breakbeam_tripped_id == getRobotId();

        // update current_robot_state
        this->current_robot_state =
            Robot(this->getRobotId(), filtered_data.position, filtered_data.velocity,
                  filtered_data.orientation, filtered_data.angular_velocity,
                  filtered_data.timestamp, breakbeam_tripped);

        return std::make_optional(this->current_robot_state);
    }
}

std::optional<Robot> RobotFilter::estimateRobotState(
    const std::vector<RobotDetection>& new_robot_data,
    const Timestamp& current_time,
    const std::optional<RobotId> breakbeam_tripped_id)
{
    // FIRST PART: GET YOUR BEST GUESS OF ACTUAL DATA
    const std::optional<RobotDetection> best_robot_detection = getBestRobotDetection(new_robot_data);

    // STEP 2: IF NOT OUT OF ORDER, DO THE PREDICTING
    if (last_predict_timestamp && current_time > *last_predict_timestamp)
    {
        predict((current_time - *last_predict_timestamp).toSeconds());
        last_predict_timestamp = current_time;
    }
    else if (!last_predict_timestamp)
    {
        last_predict_timestamp = current_time;
    }

    // if there is a good detection from above, then!
    
    if (best_robot_detection)
    {
        // check if viable, if so then update and return later?
    }
    


    return std::nullopt;
}

//completely fine DO NOT TOUCH
unsigned int RobotFilter::getRobotId() const
{
    return this->current_robot_state.id();
}

std::optional<RobotDetection> RobotFilter::getBestRobotDetection(
    const std::vector<RobotDetection>& new_robot_detections)
{
    if (new_robot_detections.empty())
    {
        return std::nullopt;
    }

    return *std::max_element(new_robot_detections.begin(),
                             new_robot_detections.end(),
                             [](const RobotDetection& a, const RobotDetection& b)
                             { return a.confidence < b.confidence; });
}

void RobotFilter::predict(double delta_t)
{
    // make a process model
    pos_kalman_filter.process_model << 1, 1, 1;
    ang_kalman_filter.process_model << 1, 1, 1;

    // make acceleration variance, position noise, correlation noise, velocity noise
    // make process covariance

    // Prediction Steps, which gets new state estimate and state covariance
    pos_kalman_filter.predict(Eigen::Vector<double, CONTROL_SIZE>::Zero());
    ang_kalman_filter.predict(Eigen::Vector<double, CONTROL_SIZE>::Zero());
}

bool RobotFilter::isWithinMaxRobotSpeed(const Point& detection_position,
                                      const Timestamp& current_time) const
{
    // within max robot speed logic
    return true;
}

void RobotFilter::reset(const PosMeasurement& pos_measurement, const AngMeasurement& ang_measurement, const Timestamp& current_time)
{
    // reset logic
}