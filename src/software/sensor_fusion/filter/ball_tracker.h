#pragma once

#include <boost/circular_buffer.hpp>
#include <optional>

#include "software/geom/line.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"
#include "software/sensor_fusion/filter/kalman_filter.h"

/**
 * Tracks a ball across frames using a Kalman filter with a constant-velocity
 * motion model. Outlier detections are rejected via Mahalanobis-distance gating;
 * if too many consecutive detections are rejected the filter is hard-reset to the
 * latest measurement.
 */
class BallTracker
{
	public:
    /**
     * Creates a new BallTracker with default noise parameters and initial state.
     */
    BallTracker();

    /**
     * Updates the tracker with the latest ball detections and returns an estimated
     * ball state.
     *
     * The best detection (highest confidence) within the filter area is selected
     * and used to update the Kalman filter. If the selected detection is an
     * outlier (Mahalanobis distance exceeds threshold), it is rejected and the
     * outlier count is incremented. Once the outlier count exceeds the threshold,
     * the filter is reset to the outlier detection.
     *
     * @param new_ball_detections A list of new ball detections from vision
     * @param filter_area The area within which ball detections are considered
     *                    valid; detections outside this area are ignored
     * @param current_time The timestamp of the current frame
     *
     * @return The estimated Ball state. If no valid detection has ever been
     *         received, returns a Ball at the origin with zero velocity.
     */
    std::optional<Ball> estimateBallState(
        const std::vector<BallDetection>& new_ball_detections,
        const Rectangle& filter_area,
		const Timestamp& current_time);

   private:
    /**
     * Selects the highest-confidence detection from the given list.
     *
     * @param new_ball_detections A list of ball detections to choose from
     *
     * @return The detection with the highest confidence, or std::nullopt if the
     *         list is empty
     */
	std::optional<BallDetection> getBestBallDetection(const std::vector<BallDetection> &new_ball_detections);
	int consecutive_outliers;
	KalmanFilter kalman_filter;
	std::optional<Timestamp> prev_detection_timestamp;
};
