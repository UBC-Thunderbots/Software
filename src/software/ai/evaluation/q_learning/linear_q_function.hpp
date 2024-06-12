#pragma once

#include <Eigen/Dense>

#include "csv.hpp"
#include "software/ai/evaluation/q_learning/feature_extractor.hpp"
#include "software/ai/evaluation/q_learning/q_function.hpp"

/**
 * Implementation of a Q-function with linear function approximation.
 *
 * We approximate the Q-function using a linear combination of state features and
 * their weights. This provides a reasonable estimate of Q(S, A) even if we have not
 * applied action A in state S previously.
 *
 * See https://gibberblot.github.io/rl-notes/single-agent/function-approximation.html
 * for more details.
 *
 * @tparam TState the type representing the state of the MDP
 * @tparam TAction the type representing the set of actions the agent can execute
 */
template <typename TState, typename TAction>
class LinearQFunction : public QFunction<TState, TAction>
{
   public:
    /**
     * Creates a LinearQFunction.
     *
     * @param features the feature extractor to use on the state representation
     * @param learning_rate the initial learning rate
     * @param discount_factor the initial discount factor
     * @param weights initial weights to use, or std::nullopt to initialize weights with 0
     */
    explicit LinearQFunction(FeatureExtractor<TState, TAction> features,
                             double learning_rate, double discount_factor,
                             std::optional<Eigen::VectorXd> weights = std::nullopt);

    /**
     * Creates a LinearQFunction with its weights initialized using the given
     * CSV file.
     *
     * @param features the feature extractor to use on the state representation
     * @param learning_rate the initial learning rate
     * @param discount_factor the initial discount factor
     * @param weights_csv_file the CSV file containing the initial weights to use
     */
    explicit LinearQFunction(FeatureExtractor<TState, TAction> features,
                             double learning_rate, double discount_factor,
                             std::string weights_csv_file);

    /**
     * Reads the given CSV file containing weights for a LinearQFunction
     * and returns the weights as a vector.
     *
     * @param csv_file the CSV file containing the weights to load
     *
     * @return the weights from the CSV file, or std::nullopt if the file is empty
     */
    static Eigen::VectorXd loadWeightsFromCsv(std::string csv_file);

    /**
     * Saves the weights of the LinearQFunction to a file in CSV format.
     * The file will be saved under tbots runtime directory (e.g. /tmp/tbots/blue).
     *
     * @param csv_file the name of the file to save the weights to
     */
    void saveWeightsToCsv(std::string csv_file);

    double getQValue(const TState& state, const TAction::Enum& action) const override;

    double getMaxQValue(const TState& state) const override;

    void update(const TState& state, const TState& next_state,
                const TAction::Enum& action, double reward) override;

    /**
     * Gets the weights of the LinearQFunction.
     *
     * @returns the function weights
     */
    const Eigen::VectorXd& getWeights();

    /**
     * Sets the learning rate used in the Q-function update equation.
     *
     * @param learning_rate the new learning rate, between 0 and 1 inclusive
     */
    void setLearningRate(double learning_rate);

    /**
     * Sets the discount factor used in the Q-function update equation.
     *
     * @param learning_rate the new discount factor, between 0 and 1 inclusive
     */
    void setDiscountFactor(double discount_factor);

   private:
    // The feature extractor to use on the state representation
    FeatureExtractor<TState, TAction> features_;

    // The weights vector with one weight for each feature-action pair
    Eigen::VectorXd weights_;

    // The learning rate used in the Q-function update equation
    double learning_rate_;

    // The discount factor used in the Q-function update equation
    double discount_factor_;
};

template <typename TState, typename TAction>
LinearQFunction<TState, TAction>::LinearQFunction(
    FeatureExtractor<TState, TAction> features, double learning_rate,
    double discount_factor, std::optional<Eigen::VectorXd> weights)
    : features_(features),
      weights_(weights.value_or(
          Eigen::VectorXd::Zero(features_.numFeatures() * TAction::numValues())))
{
    CHECK(static_cast<size_t>(weights_.size()) ==
          features_.numFeatures() * TAction::numValues())
        << "Provided LinearQFunction weights vector has wrong dimensions";

    setLearningRate(learning_rate);
    setDiscountFactor(discount_factor);
}

template <typename TState, typename TAction>
LinearQFunction<TState, TAction>::LinearQFunction(
    FeatureExtractor<TState, TAction> features, double learning_rate,
    double discount_factor, std::string weights_csv_file)
    : LinearQFunction(features, learning_rate, discount_factor,
                      loadWeightsFromCsv(weights_csv_file))
{
}

template <typename TState, typename TAction>
Eigen::VectorXd LinearQFunction<TState, TAction>::loadWeightsFromCsv(
    std::string csv_file)
{
    csv::CSVReader reader(csv_file, csv::CSVFormat().no_header());
    csv::CSVRow csv_row;
    reader.read_row(csv_row);

    Eigen::VectorXd weights(csv_row.size());
    std::transform(csv_row.begin(), csv_row.end(), weights.begin(),
                   [](csv::CSVField& field) { return field.get<double>(); });
    
    return weights;
}

template <typename TState, typename TAction>
void LinearQFunction<TState, TAction>::saveWeightsToCsv(std::string csv_file)
{
    const static Eigen::IOFormat CSV_FORMAT(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                            ",", "\n");

    LOG(CSV_OVERWRITE, csv_file) << getWeights().transpose().format(CSV_FORMAT);
}

template <typename TState, typename TAction>
double LinearQFunction<TState, TAction>::getQValue(
    const TState& state, const typename TAction::Enum& action) const
{
    Eigen::VectorXd feature_vector = features_.extract(state, action);

    return feature_vector.dot(weights_);
}

template <typename TState, typename TAction>
double LinearQFunction<TState, TAction>::getMaxQValue(const TState& state) const
{
    std::vector<typename TAction::Enum> all_actions = TAction::allValues();

    return std::transform_reduce(
        all_actions.begin(), all_actions.end(), std::numeric_limits<double>::lowest(),
        [&](auto a, auto b) { return std::max(a, b); },
        [&](const auto& action) { return getQValue(state, action); });
}

template <typename TState, typename TAction>
void LinearQFunction<TState, TAction>::update(const TState& state,
                                              const TState& next_state,
                                              const typename TAction::Enum& action,
                                              double reward)
{
    Eigen::VectorXd feature_vector = features_.extract(state, action);

    double temporal_diff_target = reward + discount_factor_ * getMaxQValue(next_state);
    double delta                = temporal_diff_target - getQValue(state, action);
    weights_                    = weights_ + (learning_rate_ * delta * feature_vector);
}

template <typename TState, typename TAction>
const Eigen::VectorXd& LinearQFunction<TState, TAction>::getWeights()
{
    return weights_;
}

template <typename TState, typename TAction>
void LinearQFunction<TState, TAction>::setLearningRate(double learning_rate)
{
    CHECK(learning_rate >= 0 && learning_rate <= 1)
        << "Q-function learning rate must be between 0 and 1 inclusive";

    learning_rate_ = learning_rate;
}

template <typename TState, typename TAction>
void LinearQFunction<TState, TAction>::setDiscountFactor(double discount_factor)
{
    CHECK(discount_factor >= 0 && discount_factor <= 1)
        << "Q-function discount factor must be between 0 and 1 inclusive";

    discount_factor_ = discount_factor;
}
