#include "software/sensor_fusion/filter/kalman_filter.hpp"

#include <gtest/gtest.h>

#include <limits>

struct KalmanParamsMath
{
    double state_estimate;
    double state_covariance;
    double process_model;
    double process_covariance;
    double control_model;
    double measurement_model;
    double measurement_covariance;
    double control_input;
    double measurement_input;
};

template <int DimX, int DimY, int DimU>
struct KalmanParamsBehaviour
{
    Eigen::Vector<double, DimX> state_estimate;
    Eigen::Matrix<double, DimX, DimX> state_covariance;
    Eigen::Matrix<double, DimX, DimX> process_model;
    Eigen::Matrix<double, DimX, DimX> process_covariance;
    Eigen::Matrix<double, DimX, DimU> control_model;
    Eigen::Matrix<double, DimY, DimX> measurement_model;
    Eigen::Matrix<double, DimY, DimY> measurement_covariance;
    Eigen::Matrix<double, DimU, 1> control_input;
    Eigen::Matrix<double, DimY, 1> measurement_input;
    Eigen::Vector<double, DimX> expected_state_estimate;
    Eigen::Matrix<double, DimX, DimX> expected_state_covariance;
};

class MathTests1D : public testing::TestWithParam<KalmanParamsMath>
{
   protected:
    static constexpr int DimX = 1;
    static constexpr int DimY = 1;
    static constexpr int DimU = 1;

    KalmanFilter<DimX, DimY, DimU> filter{};

    Eigen::Matrix<double, DimU, 1> control_input;
    Eigen::Matrix<double, DimY, 1> measurement_input;

    KalmanParamsMath p = {};

    void SetUp() override
    {
        p = GetParam();

        filter.state_estimate << p.state_estimate;
        filter.state_covariance << p.state_covariance;
        filter.process_model << p.process_model;
        filter.process_covariance << p.process_covariance;
        filter.control_model << p.control_model;
        filter.measurement_model << p.measurement_model;
        filter.measurement_covariance << p.measurement_covariance;

        // Control input
        control_input << p.control_input;
        measurement_input << p.measurement_input;
    }
};

TEST_P(MathTests1D, Math1D)
{
    // Predict Stage 1x1 Matrix Equations
    double predict_state_estimate =
        p.process_model * p.state_estimate + p.control_model * p.control_input;
    double predict_state_covariance =
        p.process_model * p.process_model * p.state_covariance + p.process_covariance;

    filter.predict(control_input);
    EXPECT_DOUBLE_EQ(filter.state_estimate(0), predict_state_estimate);
    EXPECT_DOUBLE_EQ(filter.state_covariance(0, 0), predict_state_covariance);

    // Update Stage 1x1 Matrix Equations

    double innovation_covariance =
        p.measurement_model * p.measurement_model * predict_state_covariance +
        p.measurement_covariance;
    double kalman_gain;

    if (innovation_covariance == 0)
    {
        // Kalman gain should default to 0 if the matrix is uninvertible
        kalman_gain = 0;
    }
    else
    {
        kalman_gain =
            predict_state_covariance * p.measurement_model /
            (p.measurement_model * p.measurement_model * predict_state_covariance +
             p.measurement_covariance);
    }

    double update_state_estimate =
        predict_state_estimate +
        kalman_gain *
            (p.measurement_input - p.measurement_model * predict_state_estimate);
    double update_state_covariance =
        std::pow((1 - kalman_gain * p.measurement_model), 2) * predict_state_covariance +
        std::pow(kalman_gain, 2) * p.measurement_covariance;

    filter.update(measurement_input);
    EXPECT_DOUBLE_EQ(filter.state_estimate(0), update_state_estimate);
    EXPECT_DOUBLE_EQ(filter.state_covariance(0, 0), update_state_covariance);
}

INSTANTIATE_TEST_SUITE_P(
    Math1D, MathTests1D,
    ::testing::Values(
        // Random Values
        KalmanParamsMath{1.9, 3.2, 11.7, 5.3, 2.1, 1.7, 0.2, 4.1, 5.4},

        // Negative Numbers, No Uncertainty
        KalmanParamsMath{-3.5, -0.1, -1.0, 0.0, -1.0, -1.0, 0.0, 3.4, -2.3},

        // Handles innovation covariance = 0 (matrix is uninvertible)
        KalmanParamsMath{-3.5, -0.1, -1.0, 0.0, -1.0, 0.0, 0.0, 3.4, -2.3}));

template <int DimX, int DimY, int DimU>
class BehaviourTests
    : public testing::TestWithParam<KalmanParamsBehaviour<DimX, DimY, DimU>>
{
   protected:
    KalmanFilter<DimX, DimY, DimU> filter{};

    Eigen::Vector<double, DimX> expected_state_estimate;
    Eigen::Matrix<double, DimX, DimX> expected_state_covariance;

    Eigen::Matrix<double, DimU, 1> control_input;
    Eigen::Matrix<double, DimY, 1> measurement_input;

    KalmanParamsBehaviour<DimX, DimY, DimU> p = {};

    void SetUp() override
    {
        p = this->GetParam();

        filter.state_estimate         = p.state_estimate;
        filter.state_covariance       = p.state_covariance;
        filter.process_model          = p.process_model;
        filter.process_covariance     = p.process_covariance;
        filter.control_model          = p.control_model;
        filter.measurement_model      = p.measurement_model;
        filter.measurement_covariance = p.measurement_covariance;

        // Input
        control_input     = p.control_input;
        measurement_input = p.measurement_input;

        // Expected Values
        expected_state_estimate   = p.expected_state_estimate;
        expected_state_covariance = p.expected_state_covariance;
    }
};

using BehaviourTests1D = BehaviourTests<1, 1, 1>;
TEST_P(BehaviourTests1D, Behaviour1D)
{
    filter.predict(control_input);
    filter.update(measurement_input);

    EXPECT_TRUE(filter.state_estimate.isApprox(expected_state_estimate,
                                               std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(filter.state_covariance.isApprox(expected_state_covariance,
                                                 std::numeric_limits<double>::epsilon()));
}

INSTANTIATE_TEST_SUITE_P(
    Behaviour1D, BehaviourTests1D,
    ::testing::Values(
        // High measurement noise and no control input, estimate should trust prediction
        KalmanParamsBehaviour<1, 1, 1>{
            Eigen::Vector<double, 1>{5.0}, Eigen::Matrix<double, 1, 1>{7.0},
            Eigen::Matrix<double, 1, 1>{1.0}, Eigen::Matrix<double, 1, 1>{0.0},
            Eigen::Matrix<double, 1, 1>{2.0}, Eigen::Matrix<double, 1, 1>{1.0},
            Eigen::Matrix<double, 1, 1>{1e20}, Eigen::Matrix<double, 1, 1>{0},
            Eigen::Matrix<double, 1, 1>{4.0}, Eigen::Matrix<double, 1, 1>{5.0},
            Eigen::Matrix<double, 1, 1>{7.0}},
        // High process noise, estimate should trust measurements no matter control
        KalmanParamsBehaviour<1, 1, 1>{
            Eigen::Vector<double, 1>{2.0}, Eigen::Matrix<double, 1, 1>{6.0},
            Eigen::Matrix<double, 1, 1>{1.0}, Eigen::Matrix<double, 1, 1>{1e20},
            Eigen::Matrix<double, 1, 1>{1.0}, Eigen::Matrix<double, 1, 1>{1.0},
            Eigen::Matrix<double, 1, 1>{2.0}, Eigen::Matrix<double, 1, 1>{0.0},
            Eigen::Matrix<double, 1, 1>{4.0}, Eigen::Matrix<double, 1, 1>{4.0},
            Eigen::Matrix<double, 1, 1>{2.0}},
        KalmanParamsBehaviour<1, 1, 1>{
            Eigen::Vector<double, 1>{2.0}, Eigen::Matrix<double, 1, 1>{6.0},
            Eigen::Matrix<double, 1, 1>{1.0}, Eigen::Matrix<double, 1, 1>{1e20},
            Eigen::Matrix<double, 1, 1>{1.0}, Eigen::Matrix<double, 1, 1>{1.0},
            Eigen::Matrix<double, 1, 1>{2.0}, Eigen::Matrix<double, 1, 1>{90.0},
            Eigen::Matrix<double, 1, 1>{4.0}, Eigen::Matrix<double, 1, 1>{4.0},
            Eigen::Matrix<double, 1, 1>{2.0}},
        // Equal measurement and process noise, estimate should weigh each equally
        KalmanParamsBehaviour<1, 1, 1>{
            Eigen::Vector<double, 1>{-3.4}, Eigen::Matrix<double, 1, 1>{1.0},
            Eigen::Matrix<double, 1, 1>{1.0}, Eigen::Matrix<double, 1, 1>{0.0},
            Eigen::Matrix<double, 1, 1>{0.0}, Eigen::Matrix<double, 1, 1>{1.0},
            Eigen::Matrix<double, 1, 1>{1.0}, Eigen::Matrix<double, 1, 1>{0.0},
            Eigen::Matrix<double, 1, 1>{-2.6}, Eigen::Matrix<double, 1, 1>{-3.0},
            Eigen::Matrix<double, 1, 1>{0.5}}));

using BehaviourTests2D = BehaviourTests<2, 1, 3>;
TEST_P(BehaviourTests2D, Behaviour2D)
{
    filter.predict(control_input);
    filter.update(measurement_input);

    EXPECT_TRUE(filter.state_estimate.isApprox(expected_state_estimate,
                                               std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(filter.state_covariance.isApprox(expected_state_covariance,
                                                 std::numeric_limits<double>::epsilon()));
}

INSTANTIATE_TEST_SUITE_P(
    Behaviour2D, BehaviourTests2D,
    ::testing::Values(
        // Control model of zero, control inputs should not have an effect on state
        KalmanParamsBehaviour<2, 1, 3>{
            Eigen::Vector<double, 2>{3.0, 4.0},
            Eigen::Matrix<double, 2, 2>{{7.0, 4.0}, {3.0, 5.0}},
            Eigen::Matrix<double, 2, 2>::Identity(), Eigen::Matrix<double, 2, 2>::Zero(),
            Eigen::Matrix<double, 2, 3>::Zero(), Eigen::Matrix<double, 1, 2>{{1.0, 0.0}},
            Eigen::Matrix<double, 1, 1>{1e20}, Eigen::Matrix<double, 3, 1>{300, 400, 700},
            Eigen::Matrix<double, 1, 1>{3.0}, Eigen::Matrix<double, 2, 1>{3.0, 4.0},
            Eigen::Matrix<double, 2, 2>{{7.0, 4.0}, {3.0, 5.0}}},
        // Process model and covariance of zero, the filter should forget its past
        // state and collapse to zero
        KalmanParamsBehaviour<2, 1, 3>{
            Eigen::Vector<double, 2>{0.0, 0.0},
            Eigen::Matrix<double, 2, 2>{{0.5, 0.0}, {0.0, 0.5}},
            Eigen::Matrix<double, 2, 2>::Zero(), Eigen::Matrix<double, 2, 2>::Zero(),
            Eigen::Matrix<double, 2, 3>{{1.0, 0.0, 2.0}, {1.0, 0.0, 2.0}},
            Eigen::Matrix<double, 1, 2>{{1.0, 1.0}}, Eigen::Matrix<double, 1, 1>{0.5},
            Eigen::Matrix<double, 3, 1>{0.0, 0.0, 0.0}, Eigen::Matrix<double, 1, 1>{10.0},
            Eigen::Matrix<double, 2, 1>{0.0, 0.0}, Eigen::Matrix<double, 2, 2>::Zero()},
        // Measurement model of zero, measurements should not have an effect on state
        KalmanParamsBehaviour<2, 1, 3>{
            Eigen::Vector<double, 2>{2.5, 3.5},
            Eigen::Matrix<double, 2, 2>{{0.5, 0.0}, {0.0, 0.5}},
            Eigen::Matrix<double, 2, 2>::Identity(),
            Eigen::Matrix<double, 2, 2>::Identity(),
            Eigen::Matrix<double, 2, 3>{{1.0, 0.0, 2.0}, {1.0, 0.0, 2.0}},
            Eigen::Matrix<double, 1, 2>{{0.0, 0.0}}, Eigen::Matrix<double, 1, 1>{0.5},
            Eigen::Matrix<double, 3, 1>{0.0, 0.0, 0.0},
            Eigen::Matrix<double, 1, 1>{300.0}, Eigen::Matrix<double, 2, 1>{2.5, 3.5},
            Eigen::Matrix<double, 2, 2>{{1.5, 0.0}, {0.0, 1.5}}}));

using BehaviourTests3D = BehaviourTests<3, 3, 3>;
TEST_P(BehaviourTests3D, Behaviour3D)
{
    filter.predict(control_input);
    filter.update(measurement_input);

    // Tolerance to account for the calculation drift of the larger matrix
    constexpr double tolerance = 1e-10;

    EXPECT_TRUE(filter.state_estimate.isApprox(expected_state_estimate, tolerance));
    EXPECT_TRUE(filter.state_covariance.isApprox(expected_state_covariance, tolerance));
}

INSTANTIATE_TEST_SUITE_P(
    Behaviour3D, BehaviourTests3D,
    ::testing::Values(
        // With one sensor having high measurement noise, filter should ignore prior data
        KalmanParamsBehaviour<3, 3, 3>{
            Eigen::Vector<double, 3>{1.0, 3.0, 4.0},
            Eigen::Matrix<double, 3, 3>::Identity(),
            Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero(),
            Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity(),
            Eigen::Matrix<double, 3, 3>{
                {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1e15}},
            Eigen::Vector<double, 3>{0.0, 0.0, 0.0},
            Eigen::Vector<double, 3>{5.0, 7.0, 100.0},
            Eigen::Vector<double, 3>{3.0, 5.0, 4.0},
            Eigen::Matrix<double, 3, 3>{
                {0.5, 0.0, 0.0}, {0.0, 0.5, 0.0}, {0.0, 0.0, 1.0}}},
        // With zero measurement error, the filter should trust the measurement
        KalmanParamsBehaviour<3, 3, 3>{
            Eigen::Vector<double, 3>{1.0, 3.0, 4.0},
            Eigen::Matrix<double, 3, 3>::Identity(),
            Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero(),
            Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity(),
            Eigen::Matrix<double, 3, 3>{
                {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
            Eigen::Vector<double, 3>{0.0, 0.0, 0.0},
            Eigen::Vector<double, 3>{20.0, 30.0, 50.0},
            Eigen::Vector<double, 3>{20.0, 30.0, 50.0},
            Eigen::Matrix<double, 3, 3>{
                {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}}));
