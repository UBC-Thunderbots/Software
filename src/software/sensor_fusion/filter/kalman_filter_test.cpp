#include "software/sensor_fusion/filter/kalman_filter.hpp"

#include <gtest/gtest.h>

#include <limits>

class KalmanFilterTest1D
    :  // Params: enemy robot positions, expected chip region to avoid
       public testing::TestWithParam<std::tuple<double, double, double, double, double,
                                                double, double, double, double>>
{
   protected:
    static constexpr int DimX = 1;
    static constexpr int DimY = 1;
    static constexpr int DimU = 1;

    double s_e = 0, s_c = 0, p_m = 0, p_c = 0, c_m = 0, m_m = 0, m_c = 0, c_i = 0,
           m_i = 0;

    KalmanFilter<DimX, DimY, DimU> filter{};

    Eigen::Matrix<double, DimU, 1> control_input;
    Eigen::Matrix<double, DimU, 1> measurement_input;

    void SetUp() override
    {
        s_e = std::get<0>(GetParam());
        s_c = std::get<1>(GetParam());
        p_m = std::get<2>(GetParam());
        p_c = std::get<3>(GetParam());
        c_m = std::get<4>(GetParam());
        m_m = std::get<5>(GetParam());
        m_c = std::get<6>(GetParam());
        c_i = std::get<7>(GetParam());
        m_i = std::get<8>(GetParam());

        filter.state_estimate << s_e;
        filter.state_covariance << s_c;
        filter.process_model << p_m;
        filter.process_covariance << p_c;
        filter.control_model << c_m;
        filter.measurement_model << m_m;
        filter.measurement_covariance << m_c;

        // Control input
        control_input << c_i;
        measurement_input << m_i;
    }
};

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

TEST(KalmanFilterTest, IgnoreNoisyMeasurement)
{
    constexpr int DimX = 1;
    constexpr int DimY = 1;
    constexpr int DimU = 1;

    KalmanFilter<DimX, DimY, DimU> filter{};

    filter.state_estimate << 5.0;
    filter.state_covariance << 7.0;
    filter.process_model << 1.0;
    filter.process_covariance << 0.0;
    filter.control_model << 0.0;
    filter.measurement_model << 1.0;
    filter.measurement_covariance << 1e20;  // Huge measurement noise

    // Predict with zero control input
    Eigen::Matrix<double, DimU, 1> control_input;
    control_input << 0.0;

    filter.predict(control_input);

    // Noisy measurement says state is 3
    Eigen::Matrix<double, DimY, 1> measurement;
    measurement << 3.0;

    filter.update(measurement);

    // With high measurement noise, estimate should trust prediction
    EXPECT_NEAR(filter.state_estimate(0), 5.0, std::numeric_limits<double>::epsilon());

    // Covariance should stay relatively unchanged
    EXPECT_NEAR(filter.state_covariance(0, 0), 7.0,
                std::numeric_limits<double>::epsilon());
}

TEST_P(KalmanFilterTest1D, Predict1DState)
{
    // Predict Stage
    double predict_state_estimate   = p_m * s_e + c_m * c_i;
    double predict_state_covariance = p_m * p_m * s_c + p_c;

    filter.predict(control_input);

    EXPECT_DOUBLE_EQ(filter.state_estimate(0), predict_state_estimate);
    EXPECT_DOUBLE_EQ(filter.state_covariance(0, 0), predict_state_covariance);

    double kalman_gain =
        predict_state_covariance * m_m / (m_m * m_m * predict_state_covariance + m_c);
    double update_state_estimate =
        predict_state_estimate + kalman_gain * (m_i - m_m * predict_state_estimate);
    double update_state_covariance =
        std::pow((1 - kalman_gain * m_m), 2) * predict_state_covariance +
        std::pow(kalman_gain, 2) * m_c;

    filter.update(measurement_input);
    EXPECT_DOUBLE_EQ(filter.state_estimate(0), update_state_estimate);
    EXPECT_DOUBLE_EQ(filter.state_covariance(0, 0), update_state_covariance);
}

INSTANTIATE_TEST_SUITE_P(PredictAndUpdate1D, KalmanFilterTest1D,
                         ::testing::Values(
                             // <state_estimation, state_covariance, process_model,
                             // process_covariance, control_model, measurement_model,
                             // measurement_covariance, control_input, measurement_input>
                             std::make_tuple(1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 3.0),
                             std::make_tuple(1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 4.0, 5.4),
                             std::make_tuple(2.0, 0.5, 1.0, 0.1, 1.0, 1.0, 0.2, 0.5,
                                             2.3)));
