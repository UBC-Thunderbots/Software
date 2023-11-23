#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/support_tactic_candidate.h"

#include "software/math/math_functions.h"

template<typename TSupportTactic>
SupportTacticCandidate<TSupportTactic>::SupportTacticCandidate() 
    : scores_(), total_score_(0.0), total_score_invalidated_(false)
{
}

template<typename TSupportTactic>
void SupportTacticCandidate<TSupportTactic>::score(SupportTacticScorer &scorer)
{
    double score = std::clamp(scorer.score(*this), -1.0, 1.0);
    scores_.push_back(score);
    total_score_invalidated_ = true;
}

template<typename TSupportTactic>
void SupportTacticCandidate<TSupportTactic>::resetTotalScore()
{
    scores_.clear();
    total_score_ = 0.0;
    total_score_invalidated_ = false;
}

template<typename TSupportTactic>
double SupportTacticCandidate<TSupportTactic>::getTotalScore()
{
    if (total_score_invalidated_) 
    {
        computeTotalScore();
        total_score_invalidated_ = false;
    }
    return total_score_;
}

template<typename TSupportTactic>
std::shared_ptr<TSupportTactic> SupportTacticCandidate<TSupportTactic>::create() const
{
    return std::make_shared<TSupportTactic>();
}

template<typename TSupportTactic>
void SupportTacticCandidate<TSupportTactic>::computeTotalScore()
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

    double num_scores = std::static_cast<double>(scores_.size());
    total_score_ = std::pow(sum / num_scores, SINGLE_SCORE_INFLUENCE);
}
