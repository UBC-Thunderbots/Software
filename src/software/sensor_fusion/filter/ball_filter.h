#pragma once

#include <Eigen/Dense>
#include <optional>

#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/sensor_fusion/filter/kalman_filter.hpp"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/robot.h"

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

    /**
     * Forces the estimate onto a position known from a source other than vision, such as
     * the breakbeam of a robot with the ball in its dribbler, and returns the resulting
     * ball.
     *
     * A trusted position is not a detection and must not be run through the gates that
     * detections are. Those gates compare against the current estimate, so a breakbeam
     * fed in as a detection is rejected in exactly the case it exists for -- vision has
     * lost the ball and the estimate has drifted away from where the robot says it is.
     *
     * The ball is placed at rest, since a ball held in a dribbler is not moving relative
     * to the robot holding it.
     *
     * @param position The position to force the estimate onto
     * @param current_time The time the position is valid at
     *
     * @return The ball at the forced position
     */
    Ball forceBallState(const Point& position, const Timestamp& current_time);

   private:
    // KF Dimensions
    // State: position x, position y, veloity x, velocity y
    static constexpr int STATE_SIZE = 4;
    // Measurement: x and y from vision
    static constexpr int MEASUREMENT_SIZE = 2;
    // No control
    static constexpr int CONTROL_SIZE = 1;

    using BallKalmanFilter = KalmanFilter<STATE_SIZE, MEASUREMENT_SIZE, CONTROL_SIZE>;
    using Measurement      = Eigen::Vector<double, MEASUREMENT_SIZE>;

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
     * Pulls the estimate back inside the field boundary if the motion model has pushed
     * it out, and brings it to rest against whatever it ran into.
     *
     * A ball cannot physically be outside the boundary, so an estimate that says it is
     * is wrong no matter how confident the model is. This matters most when there are no
     * detections to correct it: vision loses a ball resting against a wall, and the
     * estimate coasts straight through the boundary and keeps going for as long as the
     * ball is missing.
     *
     * The velocity component pointing out of the field is zeroed along with the position,
     * because pinning the position alone leaves a velocity that re-crosses the boundary
     * on the next frame and walks the estimate along the wall.
     *
     * @param field The field being played on
     */
    void constrainToField(const Field& field);

    /**
     * Returns whether the ball is touching anything on the field -- a robot, a goalpost,
     * the back of a net, or the walls around the field.
     *
     * The check is against the whole path the ball travelled this frame rather than only
     * where it ended up. A ball moving at 5 m/s covers over 8 cm between frames at 60 Hz,
     * so a test that only asked whether the ball was currently within its own radius of a
     * surface would step straight over anything thin, and a goalpost is thin.
     *
     * @param previous_position Where the estimate was before it was advanced this frame
     * @param robots The robots currently on the field
     * @param field The field being played on
     *
     * @return Whether the ball is in contact with anything
     */
    bool isInContact(const Point& previous_position, const std::vector<Robot>& robots,
                     const Field& field) const;

    /**
     * Corrects the motion model for a ball that has been resting against something for
     * several frames in a row by bringing it to rest.
     *
     * This runs on every frame, including frames with no detection. A ball is very often
     * occluded precisely because a robot is sitting on it, and a constant velocity model
     * left uncorrected will coast the estimate straight through that robot for as long as
     * vision cannot see it.
     *
     * A single frame of contact is not enough to conclude the ball has stopped -- a ball
     * bouncing off a wall is in contact for a frame or two and is still moving -- so the
     * estimate is only zeroed once contact has persisted.
     *
     * @param in_contact Whether the ball is touching anything this frame
     */
    void updateContactState(bool in_contact);

    /**
     * Returns whether the ball could physically have reached the given position since
     * the last accepted detection, assuming it cannot exceed the maximum ball speed.
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
     * measurement, at rest and with the covariance widened back out.
     *
     * @param measurement The measurement to reinitialize the estimate on
     * @param current_time The time the measurement was taken at
     */
    void reset(const Measurement& measurement, const Timestamp& current_time);

    BallKalmanFilter kalman_filter;
    int consecutive_outliers;
    std::optional<Timestamp> prev_detection_timestamp;
    std::optional<Measurement> prev_measurement;
    std::optional<Timestamp> last_predict_timestamp;
    int consecutive_in_contact_;
};
