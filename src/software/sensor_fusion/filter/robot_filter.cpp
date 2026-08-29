#include "software/sensor_fusion/filter/robot_filter.h"

namespace{
    constexpr double MAX_BALL_SPEED_M_PER_S = 6.0;
    constexpr double MAX_BALL_SPEED_GATE_TOLERANCE_M = 0.05;
    constexpr double MAHALANOBIS_GATE_THRESHOLD = 5;
    constexpr int CONSECUTIVE_OUTLIERS_THRESHOLD = 3;
}

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
    // Gets best detection in case camera accidentally has multiple detections
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
    
    if (best_robot_detection)
    {
        PosMeasurement pos_measurement(best_robot_detection->position.x(),
                        best_robot_detection->position.y());
        AngMeasurement ang_measurement(best_robot_detection->orientation.toRadians());

        // The first detection is all we know, so we start the estimate on it rather than
        // blending it against a state we never had grounds for
        if (!prev_detection_timestamp)
        {
            reset(pos_measurement, ang_measurement, current_time);
        }
        // Two gates determining whether we take the detection:
        // 1. Whether it is physically possible to arrive the new destination
        // 2. Statistical gating using Mahalanobis
        else if (isWithinMaxRobotSpeed(best_robot_detection->position, current_time) &&
                 pos_kalman_filter.mahalanobisDistance(pos_measurement) <
                     MAHALANOBIS_GATE_THRESHOLD)
        {
            pos_kalman_filter.update(pos_measurement);
            ang_kalman_filter.update(ang_measurement);
            consecutive_outliers     = 0;
            prev_pos_measurement     = pos_measurement;
            prev_ang_measurement     = ang_measurement;
            prev_detection_timestamp = current_time;
        }
        // If rejected, accumulate outliers. Once a threshold is reached we reset to adapt
        // to new position
        else
        {
            consecutive_outliers++;

            if (consecutive_outliers > CONSECUTIVE_OUTLIERS_THRESHOLD)
            {
                reset(pos_measurement, ang_measurement, current_time);
            }
        }
    }
    if (!prev_detection_timestamp)
    {
        return std::nullopt;
    }

    const Eigen::Vector<double, POS_STATE_SIZE> pos_state = pos_kalman_filter.state_estimate;
    const Eigen::Vector<double, ANG_STATE_SIZE> ang_state = ang_kalman_filter.state_estimate;
    const Point robot_position(pos_state(0), pos_state(1));
    const Vector robot_velocity(pos_state(2), pos_state(3));
    const Angle robot_orientation = Angle::fromRadians(ang_state(0));
    const AngularVelocity robot_angular_velocity = AngularVelocity::fromRadians(ang_state(1));
    bool breakbeam_tripped = breakbeam_tripped_id == getRobotId();
    this->current_robot_state = Robot(this->getRobotId(), robot_position, robot_velocity, robot_orientation, robot_angular_velocity, current_time, breakbeam_tripped);

    return std::make_optional(this->current_robot_state);
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
    // make a process model, these are just completely stand in variables
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
    // Without a previous detection there is no interval to reason over, so we have no
    // grounds to call this one impossible
    if (!prev_detection_timestamp)
    {
        return true;
    }

    const double delta_t = (current_time - *prev_detection_timestamp).toSeconds();
    const Point predicted_position(pos_kalman_filter.state_estimate(0),
                                   pos_kalman_filter.state_estimate(1));
    const double reachable_distance =
        MAX_BALL_SPEED_M_PER_S * std::max(delta_t, 0.0) + MAX_BALL_SPEED_GATE_TOLERANCE_M;

    return (detection_position - predicted_position).length() <= reachable_distance;
}

void RobotFilter::reset(const PosMeasurement& pos_measurement, const AngMeasurement& ang_measurement, const Timestamp& current_time)
{
    // Start the estimate at rest. Differencing two measurements to seed a velocity
    // divides vision noise by a very short timestep, and the pair either side of a
    // rejection streak is the least trustworthy pair to difference. The wide covariance
    // below lets the next few detections pull the velocity in on their own.


    // kalman_filter.state_estimate << measurement(0), measurement(1), 0, 0;
    // kalman_filter.state_covariance = INITIAL_COVARIANCE;

    consecutive_outliers = 0;
    // The reset measurement is now what the estimate is built on, so it becomes the
    // reference for the next timestep. Leaving the old timestamp here would make the
    // next predict() jump forward by the whole rejection streak.
    prev_pos_measurement         = pos_measurement;
    prev_ang_measurement         = ang_measurement;
    prev_detection_timestamp     = current_time;
    last_predict_timestamp       = current_time;
}