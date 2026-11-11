#include "software/sensor_fusion/filter/robot_filter.h"

#include "software/sensor_fusion/filter/robot_filter_constants.h"
#include "software/world/robot.h"

// control model = 0.
RobotFilter::RobotFilter(Robot current_robot_state, Duration expiry_buffer_duration)
    : current_robot_state(current_robot_state),
      pos_kalman_filter(RF_POS_INITIAL_STATE, RF_POS_INITIAL_COVARIANCE,
                        Eigen::Matrix<double, POS_STATE_SIZE, POS_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, POS_STATE_SIZE, POS_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, POS_STATE_SIZE, CONTROL_SIZE>::Zero(),
                        RF_POS_MEASUREMENT_MODEL, RF_POS_MEASUREMENT_COVARIANCE),
      ang_kalman_filter(RF_ANG_INITIAL_STATE, RF_ANG_INITIAL_COVARIANCE,
                        Eigen::Matrix<double, ANG_STATE_SIZE, ANG_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, ANG_STATE_SIZE, ANG_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, ANG_STATE_SIZE, CONTROL_SIZE>::Zero(),
                        RF_ANG_MEASUREMENT_MODEL, RF_ANG_MEASUREMENT_COVARIANCE),
      consecutive_outliers(0),
      expiry_buffer_duration(expiry_buffer_duration)
{
}
RobotFilter::RobotFilter(RobotDetection current_robot_state,
                         Duration expiry_buffer_duration)
    : current_robot_state(current_robot_state.id, current_robot_state.position,
                          Vector(0, 0), current_robot_state.orientation,
                          AngularVelocity::zero(), current_robot_state.timestamp),
      pos_kalman_filter(RF_POS_INITIAL_STATE, RF_POS_INITIAL_COVARIANCE,
                        Eigen::Matrix<double, POS_STATE_SIZE, POS_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, POS_STATE_SIZE, POS_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, POS_STATE_SIZE, CONTROL_SIZE>::Zero(),
                        RF_POS_MEASUREMENT_MODEL, RF_POS_MEASUREMENT_COVARIANCE),
      ang_kalman_filter(RF_ANG_INITIAL_STATE, RF_ANG_INITIAL_COVARIANCE,
                        Eigen::Matrix<double, ANG_STATE_SIZE, ANG_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, ANG_STATE_SIZE, ANG_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, ANG_STATE_SIZE, CONTROL_SIZE>::Zero(),
                        RF_ANG_MEASUREMENT_MODEL, RF_ANG_MEASUREMENT_COVARIANCE),
      consecutive_outliers(0),
      expiry_buffer_duration(expiry_buffer_duration)
{
}

std::optional<Robot> RobotFilter::estimateRobotState(
    const std::vector<RobotDetection>& new_robot_data, const Timestamp& current_time,
    const std::optional<RobotId> breakbeam_tripped_id)
{
    // Data older than what the filter has already processed (e.g. a stale or out of
    // order vision packet) is ignored, as the filter cannot run backwards in time
    if (last_predict_timestamp && current_time < *last_predict_timestamp)
    {
        if (!prev_detection_timestamp)
        {
            return std::nullopt;
        }
        return current_robot_state;
    }

    // Gets best detection in case camera accidentally has multiple detections
    const std::optional<RobotDetection> best_robot_detection =
        getBestRobotDetection(new_robot_data);


    // If the current_time called is ahead of current time, then runs predict step
    // Updates the last_predict_timestamp to the current time called.
    // Therefore, we must keey estimating with further points ahead.
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

        // Vision reports orientation wrapped to [-pi, pi], but the Kalman filter is
        // linear and would see a jump of ~2pi whenever the robot turns across the wrap
        // point. We unwrap the detection onto the estimate by adding clamp(new
        // orientation - predicted orientation) signed difference to it, so the innovation
        // always lies within [-pi, pi].

        const double predicted_orientation = ang_kalman_filter.state_estimate(POS_W);
        const Angle orientation_innovation = (best_robot_detection->orientation -
                                              Angle::fromRadians(predicted_orientation))
                                                 .clamp();
        AngMeasurement ang_measurement(predicted_orientation +
                                       orientation_innovation.toRadians());

        // The first detection is all we know, so we start the estimate on it rather than
        // blending it against a state we never had grounds for
        if (!prev_detection_timestamp)
        {
            reset(pos_measurement, ang_measurement, current_time);
        }
        // Two gates determining whether we take the detection:
        // 1. Whether it is physically possible to arrive the new destination at max
        // velocity
        // 2. Statistical gating using Mahalanobis distance
        // TODO: mahalanobisDistance and update both calculate pseudoinverse. is this a
        // problem?
        else if (isWithinMaxRobotSpeed(best_robot_detection->position, current_time) &&
                 pos_kalman_filter.mahalanobisDistance(pos_measurement) <
                     RF_MAHALANOBIS_GATE_THRESHOLD)
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
            if (consecutive_outliers > RF_CONSECUTIVE_OUTLIERS_THRESHOLD)
            {
                reset(pos_measurement, ang_measurement, current_time);
            }
        }
    }
    // If there isn't a best robot detection, there's been a previous detection, and it
    // has been over the expiry_buffer_duration since last update, return nullopt. This
    // will destroy the robot filter in robot_team_filter.
    else
    {
        if (prev_detection_timestamp &&
            current_time > *prev_detection_timestamp + expiry_buffer_duration)
        {
            return std::nullopt;
        }
    }
    if (!prev_detection_timestamp)
    {
        return std::nullopt;
    }

    const Eigen::Vector<double, POS_STATE_SIZE> pos_state =
        pos_kalman_filter.state_estimate;
    const Eigen::Vector<double, ANG_STATE_SIZE> ang_state =
        ang_kalman_filter.state_estimate;
    const Point robot_position(pos_state(POS_X), pos_state(POS_Y));
    const Vector robot_velocity(pos_state(VEL_X), pos_state(VEL_Y));
    const Angle robot_orientation = Angle::fromRadians(ang_state(POS_W)).clamp();
    const AngularVelocity robot_angular_velocity =
        AngularVelocity::fromRadians(ang_state(VEL_W));
    bool breakbeam_tripped = breakbeam_tripped_id == getRobotId();
    this->current_robot_state =
        Robot(this->getRobotId(), robot_position, robot_velocity, robot_orientation,
              robot_angular_velocity, current_time, breakbeam_tripped);

    return std::make_optional(this->current_robot_state);
}

unsigned int RobotFilter::getRobotId() const
{
    return this->current_robot_state.id();
}

std::optional<RobotDetection> RobotFilter::getBestRobotDetection(
    const std::vector<RobotDetection>& new_robot_detections)
{
    const unsigned int target_id = this->current_robot_state.id();
    int best_index               = -1;

    for (size_t i = 0; i < new_robot_detections.size(); ++i)
    {
        if (new_robot_detections[i].id == target_id)
        {
            if (best_index == -1 || new_robot_detections[i].confidence >
                                        new_robot_detections[best_index].confidence)
            {
                best_index = static_cast<int>(i);
            }
        }
    }

    if (best_index == -1)
    {
        return std::nullopt;
    }

    return new_robot_detections[best_index];
}

void RobotFilter::predict(double delta_t)
{
    // because robots move using motors that stay on, unlike balls that just roll, i will
    // be assuming that they keep moving with constant velocity
    // clang-format off
    pos_kalman_filter.process_model <<  1, 0, delta_t, 0,
                                        0, 1, 0, delta_t,
                                        0, 0, 1, 0,
                                        0, 0, 0, 1;
    ang_kalman_filter.process_model <<  1, delta_t,
                                        0, 1;
    // clang-format on


    // We compute position process covariance with the Discrete White Noise Acceleration
    // model. It depends on delta_t, so we compute it dynamically based on time passed
    // since last prediction
    const double acceleration_variance =
        RF_ACCELERATION_NOISE_M_PER_S_SQUARED * RF_ACCELERATION_NOISE_M_PER_S_SQUARED;
    const double delta_t_squared = delta_t * delta_t;
    const double position_noise =
        acceleration_variance * delta_t_squared * delta_t_squared / 4.0;
    const double correlation_noise =
        acceleration_variance * delta_t_squared * delta_t / 2.0;
    const double velocity_noise = acceleration_variance * delta_t_squared;

    // For angle kalman, we compute the angle process covariance with Continuous White
    // Noise Acceleration model. i don't know anymore i just pray it works.
    const double ang_acceleration_variance = RF_ANG_ACCELERATION_NOISE_RAD_PER_S_SQUARED *
                                             RF_ANG_ACCELERATION_NOISE_RAD_PER_S_SQUARED;
    const double angle_noise =
        ang_acceleration_variance * delta_t_squared * delta_t / 3.0;
    const double ang_correlation_noise =
        ang_acceleration_variance * delta_t_squared / 2.0;
    const double ang_velocity_noise = ang_acceleration_variance * delta_t;

    // clang-format off
    pos_kalman_filter.process_covariance << position_noise, 0, correlation_noise, 0,
                                            0, position_noise, 0, correlation_noise,
                                            correlation_noise, 0, velocity_noise, 0,
                                            0, correlation_noise, 0, velocity_noise;
    ang_kalman_filter.process_covariance << angle_noise, ang_correlation_noise,
                                            ang_correlation_noise, ang_velocity_noise;
    // clang-format on

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
    const Point predicted_position(pos_kalman_filter.state_estimate(POS_X),
                                   pos_kalman_filter.state_estimate(POS_Y));
    const double reachable_distance =
        RF_MAX_ROBOT_SPEED_M_PER_S * std::max(delta_t, 0.0) +
        RF_MAX_ROBOT_SPEED_GATE_TOLERANCE_M;

    return (detection_position - predicted_position).length() <= reachable_distance;
}

void RobotFilter::reset(const PosMeasurement& pos_measurement,
                        const AngMeasurement& ang_measurement,
                        const Timestamp& current_time)
{
    // Start the estimate at rest. Differencing two measurements to seed a velocity
    // divides vision noise by a very short timestep, and the pair either side of a
    // rejection streak is the least trustworthy pair to difference. The wide covariance
    // below lets the next few detections pull the velocity in on their own.


    pos_kalman_filter.state_estimate << pos_measurement(POS_X), pos_measurement(POS_Y), 0,
        0;
    ang_kalman_filter.state_estimate
        << Angle::fromRadians(ang_measurement(0)).clamp().toRadians(),
        0;
    pos_kalman_filter.state_covariance = RF_POS_INITIAL_COVARIANCE;
    ang_kalman_filter.state_covariance = RF_ANG_INITIAL_COVARIANCE;

    consecutive_outliers = 0;
    // The reset measurement is now what the estimate is built on, so it becomes the
    // reference for the next timestep. Leaving the old timestamp here would make the
    // next predict() jump forward by the whole rejection streak.
    prev_pos_measurement = pos_measurement;
    prev_ang_measurement = AngMeasurement::Constant(
        Angle::fromRadians(ang_measurement(POS_W)).clamp().toRadians());
    prev_detection_timestamp = current_time;
    last_predict_timestamp   = current_time;
}
