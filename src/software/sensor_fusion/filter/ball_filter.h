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
class BallFilter
{
	public:
    /**
     * Creates a new Ball Filter
     */
    BallFilter();

    /**
     * Update the filter with the new ball detection data, and returns the new
     * estimated state of the ball given the new data
     *
     * @param new_ball_detections A list of new Ball detections
     * @param filter_area The area within which the ball filter will work. Any detections
     * outside of this area will be ignored.
     *
     * @return The new ball based on the estimated state of the ball given the new data.
     * If a filtered result cannot be calculated, returns std::nullopt
     */
    std::optional<Ball> estimateBallState(
        const std::vector<BallDetection>& new_ball_detections,
        const Rectangle& filter_area,
		const Timestamp& current_time);

   private:
	std::optional<BallDetection> getBestBallDetection(const std::vector<BallDetection> &new_ball_detections);
	int consecutive_outliers;
	double mahalanobis_threshold=1;
	int consecutive_outliers_threshold=10;
	KalmanFilter kalman_filter;
	std::optional<Timestamp> prev_detection_timestamp;	
};
