#include "software/sensor_fusion/filter/robot_filter.h"

namespace
{
// The robot starts out unknown, so the initial estimate is given a covariance wide
// enough to cover anywhere on the field it might be and any speed it might legally be
// moving at. This makes the filter trust the first detections it sees almost
// entirely, letting it converge onto the robot within a few frames.
constexpr double INITIAL_POSITION_UNCERTAINTY_M       = 1.0;
constexpr double INITIAL_VELOCITY_UNCERTAINTY_M_PER_S = 6.5;
constexpr double INITIAL_ORIENTATION_UNCERTAINTY_RAD  = M_PI * M_PI / 3;
// TODO: do measurements for this
// not magic number trust in my mental simulation i think maximum angular velocity is like
// 2 revs per second so that's like 4pi and then variance = (w_max / 2)^2 so 4 * pi^2
// mental simulation means i imagined it btw i'll probably get a better number when i test
constexpr double INITIAL_ANG_VELOCITY_UNCERTAINTY_RAD_PER_S = 4 * M_PI * M_PI;

const Eigen::Vector<double, 4> POS_INITIAL_STATE = Eigen::Vector<double, 4>::Zero();
const Eigen::Vector<double, 2> ANG_INITIAL_STATE = Eigen::Vector<double, 2>::Zero();
const Eigen::Matrix<double, 4, 4> POS_INITIAL_COVARIANCE =
    Eigen::Vector<double, 4>(
        INITIAL_POSITION_UNCERTAINTY_M * INITIAL_POSITION_UNCERTAINTY_M,
        INITIAL_POSITION_UNCERTAINTY_M* INITIAL_POSITION_UNCERTAINTY_M,
        INITIAL_VELOCITY_UNCERTAINTY_M_PER_S* INITIAL_VELOCITY_UNCERTAINTY_M_PER_S,
        INITIAL_VELOCITY_UNCERTAINTY_M_PER_S* INITIAL_VELOCITY_UNCERTAINTY_M_PER_S)
        .asDiagonal();
const Eigen::Matrix<double, 2, 2> ANG_INITIAL_COVARIANCE =
    Eigen::Vector<double, 2>(
        INITIAL_ORIENTATION_UNCERTAINTY_RAD * INITIAL_ORIENTATION_UNCERTAINTY_RAD,
        INITIAL_ANG_VELOCITY_UNCERTAINTY_RAD_PER_S*
            INITIAL_ANG_VELOCITY_UNCERTAINTY_RAD_PER_S)
        .asDiagonal();


// How noisy we expect SSL Vision's robot position detections to be. Measure this by
// logging a stationary robot and taking the standard deviation of the detections.
// TODO: do measurements for this
constexpr double POS_VISION_NOISE_M = 0.01;
const Eigen::Matrix<double, 2, 2> POS_MEASUREMENT_COVARIANCE =
    Eigen::Matrix<double, 2, 2>::Identity() * (POS_VISION_NOISE_M * POS_VISION_NOISE_M);
constexpr double ANG_VISION_NOISE_RAD = .1;
const Eigen::Matrix<double, 1, 1> ANG_MEASUREMENT_COVARIANCE =
    Eigen::Matrix<double, 1, 1>::Identity() *
    (ANG_VISION_NOISE_RAD * ANG_VISION_NOISE_RAD);

// Vision measures robot's position, orientation but not its velocity nor angular velocity
const Eigen::Matrix<double, 2, 4> POS_MEASUREMENT_MODEL =
    (Eigen::Matrix<double, 2, 4>() << 1, 0, 0, 0, 0, 1, 0, 0).finished();
const Eigen::Matrix<double, 1, 2> ANG_MEASUREMENT_MODEL =
    (Eigen::Matrix<double, 1, 2>() << 1, 0).finished();


// TODO: test these
// The fastest we will believe the robot could be travelling when deciding whether a
// detection could plausibly belong to it.
constexpr double MAX_ROBOT_SPEED_M_PER_S = 6.0;

// Slack on the max robot speed gate, so that vision noise on a robot that has been
// sitting still cannot by itself push a detection out of reach of the estimate
constexpr double MAX_ROBOT_SPEED_GATE_TOLERANCE_M = 0.05;

// The standard deviation of the acceleration that the constant velocity motion model
// does not account for: deflections, uneven turf, and the tail of a kick. A kick
// itself is far larger than this, but it is also abrupt enough that the outlier gates
// catch it and reset the filter, so this does not need to cover one.
// TODO: this does NOT apply to a robot but like whatever man lol
constexpr double ACCELERATION_NOISE_M_PER_S_SQUARED       = 4.0;
constexpr double ANG_ACCELERATION_NOISE_RAD_PER_S_SQUARED = 1.0;

// Maximum Mahalanobi's Distance before rejecting as outlier
constexpr double MAHALANOBIS_GATE_THRESHOLD = 5;

// How many detections in a row may be rejected as outliers before we conclude the
// estimate itself is wrong and reset onto the newest detection
constexpr int CONSECUTIVE_OUTLIERS_THRESHOLD = 3;

// Number of missing robot detections the filter will tolerate before returning nullopt
// If under this number, it will return the predicted value if missing a frame
constexpr int EXPIRED_FRAME_THRESHOLD = 10;
}  // namespace
// control model = 0.
RobotFilter::RobotFilter(Robot current_robot_state)
    : current_robot_state(current_robot_state),
      pos_kalman_filter(POS_INITIAL_STATE, POS_INITIAL_COVARIANCE,
                        Eigen::Matrix<double, POS_STATE_SIZE, POS_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, POS_STATE_SIZE, POS_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, POS_STATE_SIZE, CONTROL_SIZE>::Zero(),
                        POS_MEASUREMENT_MODEL, POS_MEASUREMENT_COVARIANCE),
      ang_kalman_filter(ANG_INITIAL_STATE, ANG_INITIAL_COVARIANCE,
                        Eigen::Matrix<double, ANG_STATE_SIZE, ANG_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, ANG_STATE_SIZE, ANG_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, ANG_STATE_SIZE, CONTROL_SIZE>::Zero(),
                        ANG_MEASUREMENT_MODEL, ANG_MEASUREMENT_COVARIANCE),
      consecutive_outliers(0),
      expired_frame_count(0)
{
}
// change the constructor initializations
RobotFilter::RobotFilter(RobotDetection current_robot_state)
    : current_robot_state(current_robot_state.id, current_robot_state.position,
                          Vector(0, 0), current_robot_state.orientation,
                          AngularVelocity::zero(), current_robot_state.timestamp),
      pos_kalman_filter(POS_INITIAL_STATE, POS_INITIAL_COVARIANCE,
                        Eigen::Matrix<double, POS_STATE_SIZE, POS_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, POS_STATE_SIZE, POS_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, POS_STATE_SIZE, CONTROL_SIZE>::Zero(),
                        POS_MEASUREMENT_MODEL, POS_MEASUREMENT_COVARIANCE),
      ang_kalman_filter(ANG_INITIAL_STATE, ANG_INITIAL_COVARIANCE,
                        Eigen::Matrix<double, ANG_STATE_SIZE, ANG_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, ANG_STATE_SIZE, ANG_STATE_SIZE>::Zero(),
                        Eigen::Matrix<double, ANG_STATE_SIZE, CONTROL_SIZE>::Zero(),
                        ANG_MEASUREMENT_MODEL, ANG_MEASUREMENT_COVARIANCE),
      consecutive_outliers(0),
      expired_frame_count(0)
{
}

std::optional<Robot> RobotFilter::estimateRobotState(
    const std::vector<RobotDetection>& new_robot_data, const Timestamp& current_time,
    const std::optional<RobotId> breakbeam_tripped_id)
{
    // Gets best detection in case camera accidentally has multiple detections
    const std::optional<RobotDetection> best_robot_detection =
        getBestRobotDetection(new_robot_data);

    // If the timestamp is ahead of current time, then ignores it.
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
        AngMeasurement revolution_test(best_robot_detection->orientation.toRadians());
        // To keep Kalman filter linear, we must add revolutions. Otherwise, the Kalman
        // filter cannot process a rotation, where it would exceed 2pi and return to 0.
        if ((prev_ang_measurement.has_value()) &&
            (best_robot_detection->orientation < Angle::quarter()) &&
            (Angle::fromRadians((*prev_ang_measurement)(0)) > Angle::threeQuarter()))
        {
            ++revolutions;
        }
        if ((prev_ang_measurement.has_value()) &&
            (best_robot_detection->orientation > Angle::threeQuarter()) &&
            (Angle::fromRadians((*prev_ang_measurement)(0)) < Angle::quarter()))
        {
            --revolutions;
        }

        AngMeasurement ang_measurement(best_robot_detection->orientation.toRadians() +
                                       2 * M_PI * revolutions);

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
            expired_frame_count      = 0;
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
    else
    {
        expired_frame_count++;
        if (expired_frame_count > EXPIRED_FRAME_THRESHOLD)
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
    const Point robot_position(pos_state(0), pos_state(1));
    const Vector robot_velocity(pos_state(2), pos_state(3));
    const Angle robot_orientation = Angle::fromRadians(ang_state(0)).mod(Angle::full());
    const AngularVelocity robot_angular_velocity =
        AngularVelocity::fromRadians(ang_state(1));
    bool breakbeam_tripped = breakbeam_tripped_id == getRobotId();
    this->current_robot_state =
        Robot(this->getRobotId(), robot_position, robot_velocity, robot_orientation,
              robot_angular_velocity, current_time, breakbeam_tripped);

    return std::make_optional(this->current_robot_state);
}

// completely fine DO NOT TOUCH
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
    // be assuming that they keep moving with the same velocity
    pos_kalman_filter.process_model << 1, 0, delta_t, 0, 0, 1, 0, delta_t, 0, 0, 1, 0, 0,
        0, 0, 1;
    ang_kalman_filter.process_model << 1, delta_t, 0, 1;


    // We compute position process covariance with the Discrete White Noise Acceleration
    // model. It depends on delta_t, so we compute it dynamically based on time passed
    // since last prediction
    const double acceleration_variance =
        ACCELERATION_NOISE_M_PER_S_SQUARED * ACCELERATION_NOISE_M_PER_S_SQUARED;
    const double delta_t_squared = delta_t * delta_t;
    const double position_noise =
        acceleration_variance * delta_t_squared * delta_t_squared / 4.0;
    const double correlation_noise =
        acceleration_variance * delta_t_squared * delta_t / 2.0;
    const double velocity_noise = acceleration_variance * delta_t_squared;

    // For angle kalman, we compute the angle process covariance with Continuous White
    // Noise Acceleration model. i don't know anymore i just pray it works.
    const double ang_acceleration_variance = ANG_ACCELERATION_NOISE_RAD_PER_S_SQUARED *
                                             ANG_ACCELERATION_NOISE_RAD_PER_S_SQUARED;
    const double angle_noise =
        ang_acceleration_variance * delta_t_squared * delta_t / 3.0;
    const double ang_correlation_noise =
        ang_acceleration_variance * delta_t_squared / 2.0;
    const double ang_velocity_noise = ang_acceleration_variance * delta_t;

    pos_kalman_filter.process_covariance << position_noise, 0, correlation_noise, 0, 0,
        position_noise, 0, correlation_noise, correlation_noise, 0, velocity_noise, 0, 0,
        correlation_noise, 0, velocity_noise;
    ang_kalman_filter.process_covariance << angle_noise, ang_correlation_noise,
        ang_correlation_noise, ang_velocity_noise;

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
    const double reachable_distance = MAX_ROBOT_SPEED_M_PER_S * std::max(delta_t, 0.0) +
                                      MAX_ROBOT_SPEED_GATE_TOLERANCE_M;

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


    pos_kalman_filter.state_estimate << pos_measurement(0), pos_measurement(1), 0, 0;
    ang_kalman_filter.state_estimate
        << Angle::fromRadians(ang_measurement(0)).mod(Angle::full()).toRadians(),
        0;
    pos_kalman_filter.state_covariance = POS_INITIAL_COVARIANCE;
    ang_kalman_filter.state_covariance = ANG_INITIAL_COVARIANCE;

    revolutions          = 0;
    consecutive_outliers = 0;
    expired_frame_count  = 0;
    // The reset measurement is now what the estimate is built on, so it becomes the
    // reference for the next timestep. Leaving the old timestamp here would make the
    // next predict() jump forward by the whole rejection streak.
    prev_pos_measurement = pos_measurement;
    prev_ang_measurement = AngMeasurement::Constant(
        Angle::fromRadians(ang_measurement(0)).mod(Angle::full()).toRadians());
    prev_detection_timestamp = current_time;
    last_predict_timestamp   = current_time;
}
