#include "software/sensor_fusion/filter/kalman_filter.hpp"

#include <gtest/gtest.h>

#include <limits>

TEST(KalmanFilterTest, BasicScalarUpdate)
{
    constexpr int DimX = 1;
    constexpr int DimY = 1;
    constexpr int DimU = 1;

    KalmanFilter<DimX, DimY, DimU> filter{};

    filter.state_estimate << 1.0;
    filter.state_covariance << 1.0;
    filter.process_model << 1.0;
    filter.process_covariance << 0.0;
    filter.control_model << 0.0;
    filter.measurement_model << 1.0;
    filter.measurement_covariance << 1.0;

    // Predict with zero control input
    Eigen::Matrix<double, DimU, 1> control_input;
    control_input << 0.0;

    filter.predict(control_input);

    // Measurement says state is 3
    Eigen::Matrix<double, DimY, 1> measurement;
    measurement << 3.0;

    filter.update(measurement);

    // With equal prior and measurement uncertainty:
    // estimate should move halfway toward measurement
    EXPECT_NEAR(filter.state_estimate(0), 2.0, std::numeric_limits<double>::epsilon());

    // Covariance should reduce from 1 -> 0.5
    EXPECT_NEAR(filter.state_covariance(0, 0), 0.5,
                std::numeric_limits<double>::epsilon());
}
