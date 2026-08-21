#pragma once

#include <Eigen/Dense>
#include <optional>
#include <string_view>
#include <utility>

#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/geom/segment.h"
#include "software/world/field.h"
#include "software/sensor_fusion/filter/kalman_filter.hpp"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"
#include "software/world/robot.h"

/**
 * Given ball data from SSL Vision, filters and returns the position/velocity of the
 * "real" ball.
 *
 * This filter tracks the ball with a Kalman filter over the state
 * [x, y, x_velocity, y_velocity], using a constant velocity motion model with a damping
 * term to account for the ball decelerating due to friction. Each frame it takes the
 * highest confidence detection inside the filter area as the measurement.
 *
 * The data we receive isn't perfect, which is why we have a filter. A detection is
 * rejected as an outlier if it fails either of two gates: a physical one, asking whether
 * the ball could have travelled that far since the last accepted detection at all, and a
 * statistical one (Mahalanobis distance, which accounts for how confident the filter
 * currently is). The physical gate does not depend on the covariance being well tuned,
 * so it still catches nonsense while the statistical gate is what keeps the output
 * steady. Steadiness matters because small deviations in velocity orientation have large
 * effects when the AI predicts the ball's future position. For example, consistently
 * receiving a pass relies on the ball's velocity being stable, otherwise the robot would
 * "jiggle" back and forth as the estimated receiver position kept changing.
 *
 * Rejecting outliers forever would leave the filter stuck if the ball genuinely
 * teleports (a ball placement, or a detection we had wrongly locked onto), so after
 * enough consecutive rejections the filter resets onto the newest detection.
 *
 * A ball in contact with anything -- a robot, a goalpost, the back of a net, the walls
 * around the field -- has stopped following the motion model, and we no longer have
 * grounds for the confidence the covariance claims. So before considering the new
 * detection the filter looks for contact, widens the covariance back out when it finds
 * any, and reflects the velocity off the surface that was hit.
 */
class BallFilter
{
   public:
    /**
     * Creates a new Ball Filter
     */
    explicit BallFilter();

    /**
     * Update the filter with the new ball detection data, and returns the new
     * estimated state of the ball given the new data
     *
     * @param new_ball_detections A list of new Ball detections
     * @param field The field being played on. Detections outside its boundary are
     * ignored, and its goals are obstacles the ball may bounce off.
     * @param robots The robots currently on the field, which the ball may bounce off
     * @param current_time The time to estimate the ball's state at
     *
     * @return The new ball based on the estimated state of the ball given the new data.
     * If a filtered result cannot be calculated, returns std::nullopt
     */
    std::optional<Ball> estimateBallState(
        const std::vector<BallDetection>& new_ball_detections, const Field& field,
        const std::vector<Robot>& robots, const Timestamp& current_time);

   private:
    // The dimensions of the Kalman filter this ball filter is built on.
    // The state is [x, y, x_velocity, y_velocity], the measurement is [x, y], and
    // there is no control input (we cannot command the ball).
    static constexpr int STATE_SIZE       = 4;
    static constexpr int MEASUREMENT_SIZE = 2;
    static constexpr int CONTROL_SIZE     = 1;

    using BallKalmanFilter =
        KalmanFilter<STATE_SIZE, MEASUREMENT_SIZE, CONTROL_SIZE>;
    using Measurement = Eigen::Vector<double, MEASUREMENT_SIZE>;

    /**
     * Returns the detection we should treat as the ball this frame, which is the
     * highest confidence detection lying inside the filter area.
     *
     * @param new_ball_detections The detections to choose from
     * @param filter_area The area within which the ball filter will work. Any detections
     * outside of this area will be ignored.
     *
     * @return The detection to use, or std::nullopt if there is no usable detection
     */
    static std::optional<BallDetection> getBestBallDetection(
        const std::vector<BallDetection>& new_ball_detections,
        const Rectangle& filter_area);

    /**
     * Advances the Kalman filter's estimate forward to the given time using a constant
     * velocity motion model with damping.
     *
     * Both the motion model and the process noise depend on how much time is being
     * advanced over, so both are rebuilt here rather than being fixed at construction.
     *
     * @param delta_t The amount of time to advance the estimate by, in seconds
     */
    void predict(double delta_t);

    /**
     * Widens the covariance if the estimated position is in contact with anything, and
     * reflects the estimated velocity off whatever it hit.
     *
     * The covariance is widened on any contact, including one we cannot usefully reflect
     * off, because a ball touching another object is no longer described by the motion
     * model and the filter has no business staying as confident as it was.
     *
     * @param previous_position Where the estimate was before it was advanced this frame
     * @param robots The robots currently on the field
     * @param field The field being played on
     */
    void handleCollisions(const Point& previous_position,
                          const std::vector<Robot>& robots, const Field& field);

    // A contact between the ball and something else on the field
    struct Collision
    {
        // The outward normal of the surface that was hit
        Vector normal;
        // What was hit, for logging
        std::string_view object;
    };

    /**
     * Returns the contact between the ball and the field, if the ball hit anything over
     * the given path or has come to rest against it.
     *
     * The test is against the whole path the ball travelled this frame rather than only
     * where it ended up. A ball moving at 5 m/s covers over 8 cm between frames at 60 Hz,
     * so a test that only asked whether the ball was currently within its own radius of a
     * surface would step straight over anything thin, and a goalpost is thin.
     *
     * @param ball_path The path the ball travelled over this timestep
     * @param robots The robots currently on the field
     * @param field The field being played on
     *
     * @return The contact, or std::nullopt if the ball did not hit anything
     */
    static std::optional<Collision> getCollision(const Segment& ball_path,
                                                 const std::vector<Robot>& robots,
                                                 const Field& field);

    /**
     * Returns every surface on the field the ball can bounce off, paired with a name for
     * logging.
     *
     * A goal is a frame rather than a box: its mouth is an opening the ball travels
     * through, so only the back of the net and the two posts are barriers.
     *
     * @param field The field being played on
     *
     * @return The barrier segments and their names
     */
    static std::vector<std::pair<Segment, std::string_view>> getBarriers(
        const Field& field);

    /**
     * Returns whether the ball could physically have reached the given position since
     * the last accepted detection, assuming it cannot exceed the maximum ball speed.
     *
     * Unlike the Mahalanobis gate this does not depend on the covariance being well
     * tuned, so it still rejects impossible detections when the filter's own sense of
     * its uncertainty is wrong.
     *
     * @param detection_position The position of the detection to check
     * @param current_time The time the detection was taken at
     *
     * @return whether the detection is within reach of the current estimate
     */
    bool isWithinMaxBallSpeed(const Point& detection_position,
                              const Timestamp& current_time) const;

    /**
     * Discards the filter's current estimate and reinitializes it on the given
     * measurement, seeding the velocity from the previous measurement if we have one.
     *
     * @param measurement The measurement to reinitialize the estimate on
     * @param current_time The time the measurement was taken at
     */
    void reset(const Measurement& measurement, const Timestamp& current_time);

    BallKalmanFilter kalman_filter;
    // How many detections in a row have been rejected as outliers
    int consecutive_outliers;
    // The time and value of the most recent accepted measurement. Both are unset until
    // the filter has seen its first usable detection.
    std::optional<Timestamp> prev_detection_timestamp;
    std::optional<Measurement> prev_measurement;
    // The time the estimate has already been advanced to. This tracks every predict,
    // including the ones on frames where no detection was accepted, so that coasting
    // through a gap in the detections advances the estimate by the elapsed time exactly
    // once rather than re-integrating the whole gap on every frame.
    std::optional<Timestamp> last_predict_timestamp;
};
