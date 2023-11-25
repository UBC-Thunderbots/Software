#include "software/ai/evaluation/scoring/candidate.h"

#include <numeric>

#include "software/math/math_functions.h"

Candidate::Candidate() 
    : scores_(), total_score_(0), total_score_invalidated_(false)
{
}

double Candidate::getTotalScore()
{
    if (total_score_invalidated_) 
    {
        computeTotalScore();
        total_score_invalidated_ = false;
    }
    return total_score_;
}

void Candidate::applyScore(double score)
{
    scores_.push_back(std::clamp(score, -1.0, 1.0));
    total_score_invalidated_ = true;
}

void Candidate::clearScores()
{
    scores_.clear();
    total_score_ = 0;
    total_score_invalidated_ = false;
}

void Candidate::computeTotalScore()
{
    if (scores_.empty())
    {
        total_score_ = 0;
        return;
    }

    double sum = std::accumulate(scores_.begin(), scores_.end(), 0.0,
        [](double current_sum, double score) {
            double term = signum(score) * std::pow(std::abs(score), 1.0 / SINGLE_SCORE_INFLUENCE);
            return current_sum + term;
        });

    double num_scores = static_cast<double>(scores_.size());
    total_score_ = std::pow(sum / num_scores, SINGLE_SCORE_INFLUENCE);
}
