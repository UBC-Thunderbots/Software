#include "software/sensor_fusion/filter/ball_filter.h"

#include <algorithm>
#include <vector>

#include "software/geom/algorithms/contains.h"

namespace
{
    // The ball starts out completely unknown, so the initial estimate is given a very
    // large covariance. This makes the filter trust the first detections it sees almost
    // entirely, letting it converge onto the ball within a few frames.
    const Eigen::Vector<double, 4> INITIAL_STATE      = Eigen::Vector<double, 4>::Zero();
    const Eigen::Matrix<double, 4, 4> INITIAL_COVARIANCE =
        Eigen::Matrix<double, 4, 4>::Identity() * 1000.0 ;

    // How much we expect the constant velocity motion model to be wrong by. The position
    // and velocity terms are correlated because an error in velocity accumulates into an
    // error in position over the timestep.
  // σ_a = 5 m/s², discretized at Δt = 1/60 s
  const Eigen::Matrix<double, 4, 4> PROCESS_COVARIANCE =
      (Eigen::Matrix<double, 4, 4>() << 4.823e-7, 0, 5.787e-5, 0, 0, 4.823e-7, 0,
       5.787e-5, 5.787e-5, 0, 6.944e-3, 0, 0, 5.787e-5, 0, 6.944e-3)
          .finished();

  // σ_z = 2 cm, isotropic
  const Eigen::Matrix<double, 2, 2> MEASUREMENT_COVARIANCE =
      Eigen::Matrix<double, 2, 2>::Identity() * 4.0e-4;
    // Vision measures the ball's position but not its velocity
    const Eigen::Matrix<double, 2, 4> MEASUREMENT_MODEL =
        (Eigen::Matrix<double, 2, 4>() << 1, 0, 0, 0, 0, 1, 0, 0).finished();

    // The fraction of its velocity the ball retains each second as it rolls, accounting
    // for friction. Empirically measured.
    constexpr double DAMPING = 0.9889;

    // Detections whose squared Mahalanobis distance from the current estimate exceeds
    // this are treated as outliers and not fed to the filter
    constexpr double MAHALANOBIS_GATE_THRESHOLD = 5;

    // How many detections in a row may be rejected as outliers before we conclude the
    // estimate itself is wrong and reset onto the newest detection
    constexpr int CONSECUTIVE_OUTLIERS_THRESHOLD = 3;
}  // namespace

BallFilter::BallFilter()
    : kalman_filter(INITIAL_STATE, INITIAL_COVARIANCE,
                    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity(),
                    PROCESS_COVARIANCE,
                    Eigen::Matrix<double, STATE_SIZE, CONTROL_SIZE>::Zero(),
                    MEASUREMENT_MODEL, MEASUREMENT_COVARIANCE),
      consecutive_outliers(0)
{
}

std::optional<Ball> BallFilter::estimateBallState(
    const std::vector<BallDetection>& new_ball_detections, const Rectangle& filter_area,
    const Timestamp& current_time)
{
    const std::optional<BallDetection> best_ball_detection =
        getBestBallDetection(new_ball_detections, filter_area);

    // Coast the estimate forward to the current time. Doing this before considering the
    // new detection means the filter keeps producing a sensible ball even on frames
    // where the ball is occluded and there is no detection at all.
    if (prev_detection_timestamp)
    {
        predict((current_time - *prev_detection_timestamp).toSeconds());
    }

    if (best_ball_detection)
    {
        Measurement measurement(best_ball_detection->position.x(),
                                best_ball_detection->position.y());

        if (kalman_filter.mahalanobisDistance(measurement) < MAHALANOBIS_GATE_THRESHOLD)
        {
            kalman_filter.update(measurement);
            consecutive_outliers     = 0;
            prev_measurement         = measurement;
            prev_detection_timestamp = current_time;
        }
        else
        {
            consecutive_outliers++;

            // We have rejected too many detections in a row to still believe our own
            // estimate, so throw it away and start again from what vision is telling us
            if (consecutive_outliers > CONSECUTIVE_OUTLIERS_THRESHOLD)
            {
                reset(measurement, current_time);
            }
        }
    }

    // The filter has never seen a usable detection, so it has nothing to report. Vision
    // has not told us where the ball is even once, and the zeroed initial state is not a
    // real estimate.
    if (!prev_detection_timestamp)
    {
        return std::nullopt;
    }

    const Eigen::Vector<double, STATE_SIZE> state = kalman_filter.state_estimate;
    const Point ball_position(state(0), state(1));
    const Vector ball_velocity(state(2), state(3));
    const double distance_from_ground =
        best_ball_detection ? best_ball_detection->distance_from_ground : 0.0;

    return Ball(BallState(ball_position, ball_velocity, distance_from_ground),
                current_time);
}

std::optional<BallDetection> BallFilter::getBestBallDetection(
    const std::vector<BallDetection>& new_ball_detections, const Rectangle& filter_area)
{
    std::vector<BallDetection> detections_in_filter_area;
    std::copy_if(new_ball_detections.begin(), new_ball_detections.end(),
                 std::back_inserter(detections_in_filter_area),
                 [&filter_area](const BallDetection& detection)
                 { return contains(filter_area, detection.position); });

    if (detections_in_filter_area.empty())
    {
        return std::nullopt;
    }

    return *std::max_element(detections_in_filter_area.begin(),
                             detections_in_filter_area.end(),
                             [](const BallDetection& a, const BallDetection& b)
                             { return a.confidence < b.confidence; });
}

void BallFilter::predict(double delta_t)
{
    // Constant velocity motion model, with the ball's velocity decaying by the damping
    // term as it rolls
    kalman_filter.process_model << 1, 0, delta_t, 0, 0, 1, 0, delta_t, 0, 0, DAMPING, 0,
        0, 0, 0, DAMPING;

    kalman_filter.predict(Eigen::Vector<double, CONTROL_SIZE>::Zero());
}

void BallFilter::reset(const Measurement& measurement, const Timestamp& current_time)
{
    // If we have a previous measurement to compare against we can seed a velocity from
    // it, otherwise we can only say where the ball is and not where it is going
    Vector velocity(0, 0);
    if (prev_measurement && prev_detection_timestamp)
    {
        const double delta_t = (current_time - *prev_detection_timestamp).toSeconds();
        if (delta_t > 0)
        {
            velocity = Vector((measurement(0) - (*prev_measurement)(0)) / delta_t,
                              (measurement(1) - (*prev_measurement)(1)) / delta_t);
        }
    }

    kalman_filter.state_estimate << measurement(0), measurement(1), velocity.x(),
        velocity.y();
    kalman_filter.state_covariance = INITIAL_COVARIANCE;

    consecutive_outliers = 0;
    // The reset measurement is now what the estimate is built on, so it becomes the
    // reference for the next timestep. Leaving the old timestamp here would make the
    // next predict() jump forward by the whole rejection streak.
    prev_measurement         = measurement;
    prev_detection_timestamp = current_time;
}
