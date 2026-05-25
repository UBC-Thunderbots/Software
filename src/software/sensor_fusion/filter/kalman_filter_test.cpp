#include "software/sensor_fusion/filter/kalman_filter.hpp"

#include <gtest/gtest.h>

#include <limits>

// For clarity
template <int x, int y>
using matrix = Eigen::Matrix<double, x, y>;

template <int dim>
using vec = Eigen::Vector<double, dim>;

struct KalmanParamsMath
{
    /* Params: <state_estimation, state_covariance, process_model,
                            process_covariance, control_model, measurement_model,
                            measurement_covariance, control_input, measurement_input> */

    // Filter Params
    double s_e;  // State estimation
    double s_c;  // State covariance
    double p_m;  // Process Mmodel
    double p_c;  // Process Covariance
    double c_m;  // Control Model
    double m_m;  // Measurement Model
    double m_c;  // Measurement Covariance

    // Input
    double c_i;  // Control Input
    double m_i;  // Measurement Input
};

template <int DimX, int DimY, int DimU>
struct KalmanParamsBehaviour
{
    /* Params: <state_estimation, state_covariance, process_model,
                        process_covariance, control_model, measurement_model,
                        measurement_covariance, control_input, measurement_input> */

    // Filter Params
    vec<DimX> state_estimate;
    matrix<DimX, DimX> state_covariance;
    matrix<DimX, DimX>  process_model;
    matrix<DimX, DimX>  process_covariance;
    matrix<DimX, DimU>  control_model;
    matrix<DimY, DimX>  measurement_model;
    matrix<DimY, DimY>  measurement_covariance;

    // Input
    matrix<DimU, 1> control_input;
    matrix<DimU, 1> measurement_input;

    // Expected Values
    vec<DimX> expected_state_estimate;
    matrix<DimX, DimX> expected_state_covariance;

};

class MathTests1D :  // Params: Kalman Params in 1D
                     public testing::TestWithParam<KalmanParamsMath>
{
   protected:
    static constexpr int DimX = 1;
    static constexpr int DimY = 1;
    static constexpr int DimU = 1;

    KalmanFilter<DimX, DimY, DimU> filter{};

    matrix<DimU, 1> control_input;
    matrix<DimU, 1> measurement_input;

    KalmanParamsMath p = {};

    void SetUp() override
    {
        p = GetParam();

        filter.state_estimate << p.s_e;
        filter.state_covariance << p.s_c;
        filter.process_model << p.p_m;
        filter.process_covariance << p.p_c;
        filter.control_model << p.c_m;
        filter.measurement_model << p.m_m;
        filter.measurement_covariance << p.m_c;

        // Control input
        control_input << p.c_i;
        measurement_input << p.m_i;
    }
};

template <int DimX, int DimY, int DimU>
class BehaviourTests
    :  // Params: Kalman Params as Matrix, expected state, expected covariance
                    public testing::TestWithParam<KalmanParamsBehaviour<DimX, DimY, DimU>>
{
   protected:

    KalmanFilter<DimX, DimY, DimU> filter{};

    vec<DimX> expected_state_estimate;
    matrix<DimX, DimX> expected_state_covariance;

    matrix<DimU, 1> control_input;
    matrix<DimU, 1> measurement_input;

    KalmanParamsBehaviour<DimX, DimY, DimU> p = {};

    void SetUp() override
    {
        p = this->GetParam();

        filter.state_estimate = p.state_estimate;
        filter.state_covariance = p.state_covariance;
        filter.process_model = p.process_model;
        filter.process_covariance = p.process_covariance;
        filter.control_model = p.control_model;
        filter.measurement_model = p.measurement_model;
        filter.measurement_covariance = p.measurement_covariance;

        // Input
        control_input = p.control_input;
        measurement_input = p.measurement_input;

        // Expected Values
        expected_state_estimate =  p.expected_state_estimate;
        expected_state_covariance = p.expected_state_covariance;
    }
};

// Mathematical Checks

TEST_P(MathTests1D, Math1D)
{
    // Predict Stage
    double predict_state_estimate   = p.p_m * p.s_e + p.c_m * p.c_i;
    double predict_state_covariance = p.p_m * p.p_m * p.s_c + p.p_c;

    filter.predict(control_input);

    EXPECT_DOUBLE_EQ(filter.state_estimate(0), predict_state_estimate);
    EXPECT_DOUBLE_EQ(filter.state_covariance(0, 0), predict_state_covariance);

    double kalman_gain = predict_state_covariance * p.m_m /
                         (p.m_m * p.m_m * predict_state_covariance + p.m_c);
    double update_state_estimate =
        predict_state_estimate + kalman_gain * (p.m_i - p.m_m * predict_state_estimate);
    double update_state_covariance =
        std::pow((1 - kalman_gain * p.m_m), 2) * predict_state_covariance +
        std::pow(kalman_gain, 2) * p.m_c;

    filter.update(measurement_input);
    EXPECT_DOUBLE_EQ(filter.state_estimate(0), update_state_estimate);
    EXPECT_DOUBLE_EQ(filter.state_covariance(0, 0), update_state_covariance);
}

INSTANTIATE_TEST_SUITE_P(
    Math1D, MathTests1D,
    ::testing::Values(
        /* <state_estimation, state_covariance, process_model,
         process_covariance, control_model, measurement_model,
         measurement_covariance, control_input, measurement_input> */

        // Random Vals
        KalmanParamsMath{1.9, 3.2, 11.7, 5.3, 2.1, 1.7, 0.2, 4.1, 5.4},

        // Negative Numbers, No Uncertainty
        KalmanParamsMath{-3.5, -0.1, -1.0, 0.0, -1.0, -1.0, 0.0, 3.4, -2.3}));

// Behavioural Tests

using BehaviourTests1D = BehaviourTests<1, 1, 1>;
TEST_P(BehaviourTests1D, Behaviour1D)
{
    filter.predict(control_input);
    filter.update(measurement_input);

    EXPECT_TRUE(filter.state_estimate.isApprox(expected_state_estimate, std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(filter.state_covariance.isApprox(expected_state_covariance, std::numeric_limits<double>::epsilon()));

}

INSTANTIATE_TEST_SUITE_P(
    Behaviour1D, BehaviourTests1D,
    ::testing::Values(
        // High measurement noise and no control input, estimate should trust prediction
        KalmanParamsBehaviour<1,1,1>{vec<1>{5.0}, matrix<1, 1>{7.0}, matrix<1, 1>{1.0}, matrix<1, 1>{0.0}, matrix<1, 1>{0.0}, matrix<1, 1>{1.0}, matrix<1, 1>{1e20}, matrix<1, 1>{0}, matrix<1, 1>{4.0}, matrix<1, 1>{5.0}, matrix<1, 1>{7.0}},

        // High process noise, estimate should trust measurements no matter control
        KalmanParamsBehaviour<1,1,1>{vec<1>{2.0}, matrix<1, 1>{6.0}, matrix<1, 1>{1.0}, matrix<1, 1>{1e20}, matrix<1, 1>{1.0}, matrix<1, 1>{1.0}, matrix<1, 1>{2.0}, matrix<1, 1>{0.0}, matrix<1, 1>{4.0}, matrix<1, 1>{4.0}, matrix<1, 1>{2.0}},
        KalmanParamsBehaviour<1,1,1>{vec<1>{2.0}, matrix<1, 1>{6.0}, matrix<1, 1>{1.0}, matrix<1, 1>{1e20}, matrix<1, 1>{1.0}, matrix<1, 1>{1.0}, matrix<1, 1>{2.0}, matrix<1, 1>{90.0}, matrix<1, 1>{4.0}, matrix<1, 1>{4.0}, matrix<1, 1>{2.0}},

        // Equal measurement and process noise, estimate should weigh each equally
        KalmanParamsBehaviour<1,1,1>{vec<1>{-3.4}, matrix<1,1>{1.0}, matrix<1,1>{1.0}, matrix<1,1>{0.0}, matrix<1, 1>{0.0}, matrix<1,1>{1.0}, matrix<1,1>{1.0}, matrix<1,1>{0.0}, matrix<1,1>{-2.6}, matrix<1,1>{-3.0}, matrix<1,1>{0.5}}

    ));